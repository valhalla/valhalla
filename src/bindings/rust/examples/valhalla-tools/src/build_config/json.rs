//! JSON with Python's `json` semantics: what `json.load` reads and `json.dumps(indent=2)` writes.

use std::fmt::Write as _;

use indexmap::IndexMap;

use crate::python::{INT_MAX_STR_DIGITS, float_repr, normalized_int};

/// Deepest nesting read. Python's own limit comes from its stack, beyond 30 000 levels; parsing,
/// dumping and dropping recurse here, and an unoptimized build overflows the 8 MiB main-thread
/// stack around 1 600 levels, so this keeps a margin. Valhalla configs nest about 5 levels.
pub const MAX_DEPTH: usize = 1_000;

/// A Python value as `json.load` builds it.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Null,
    Bool(bool),
    /// Normalized decimal digits, as Python ints have no size limit.
    Int(String),
    Float(f64),
    Str(String),
    List(Vec<Value>),
    /// Insertion-ordered like a Python dict: a repeated key keeps its first place and last value.
    Dict(IndexMap<String, Value>),
}

/// Why a document could not be read; the `Display` of [`ParseError::Syntax`] is Python's.
#[derive(Debug, thiserror::Error, PartialEq)]
pub enum ParseError {
    /// Python's `JSONDecodeError`.
    #[error("{message}: line {line} column {column} (char {position})")]
    Syntax {
        message: &'static str,
        line: usize,
        column: usize,
        position: usize,
    },
    /// A `\uD800`-`\uDFFF` escape without its pair: Python accepts it, Rust strings cannot hold it.
    #[error("unpaired surrogate escape: line {line} column {column} (char {position})")]
    LoneSurrogate {
        line: usize,
        column: usize,
        position: usize,
    },
    #[error("Exceeds the limit ({INT_MAX_STR_DIGITS} digits) for integer string conversion")]
    IntTooLong,
    #[error(
        "nesting deeper than {MAX_DEPTH} levels is not supported: line {line} column {column} \
         (char {position})"
    )]
    TooDeep {
        line: usize,
        column: usize,
        position: usize,
    },
}

/// `json.loads` of the text Python reads from a file in text mode, where any `\r\n` or `\r` is
/// already a `\n`; positions in errors count characters, as Python's do.
pub fn parse(text: &str) -> Result<Value, ParseError> {
    let text = text.replace("\r\n", "\n").replace('\r', "\n");
    let chars: Vec<char> = text.chars().collect();
    let mut parser = Parser {
        chars: &chars,
        depth: 0,
    };
    if chars.first() == Some(&'\u{feff}') {
        return Err(parser.syntax("Unexpected UTF-8 BOM (decode using utf-8-sig)", 0));
    }
    let start = parser.skip_whitespace(0);
    let (value, end) = parser
        .value(start)?
        .ok_or_else(|| parser.syntax("Expecting value", start))?;
    let end = parser.skip_whitespace(end);
    if end != chars.len() {
        return Err(parser.syntax("Extra data", end));
    }
    Ok(value)
}

struct Parser<'a> {
    chars: &'a [char],
    depth: usize,
}

/// A scanned value and the index after it.
type Scanned = Option<(Value, usize)>;

impl Parser<'_> {
    fn at(&self, index: usize) -> Option<char> {
        self.chars.get(index).copied()
    }

    fn location(&self, position: usize) -> (usize, usize) {
        let before = &self.chars[..position.min(self.chars.len())];
        let line = before.iter().filter(|&&c| c == '\n').count() + 1;
        let column = match before.iter().rposition(|&c| c == '\n') {
            Some(newline) => position - newline,
            None => position + 1,
        };
        (line, column)
    }

    fn syntax(&self, message: &'static str, position: usize) -> ParseError {
        let (line, column) = self.location(position);
        ParseError::Syntax {
            message,
            line,
            column,
            position,
        }
    }

    fn skip_whitespace(&self, mut index: usize) -> usize {
        while matches!(self.at(index), Some(' ' | '\t' | '\n' | '\r')) {
            index += 1;
        }
        index
    }

    /// The value at `index`, or `None` where Python's scanner raises `StopIteration`, which its
    /// callers report as "Expecting value" at `index`.
    fn value(&mut self, index: usize) -> Result<Scanned, ParseError> {
        let rest = &self.chars[index.min(self.chars.len())..];
        let constant = |word: &str, value: Value| {
            let length = word.chars().count();
            (rest.len() >= length && rest[..length].iter().copied().eq(word.chars()))
                .then_some((value, index + length))
        };
        Ok(match self.at(index) {
            Some('"') => {
                let (text, end) = self.string(index + 1)?;
                Some((Value::Str(text), end))
            }
            Some(open @ ('{' | '[')) => {
                if self.depth == MAX_DEPTH {
                    let (line, column) = self.location(index);
                    return Err(ParseError::TooDeep {
                        line,
                        column,
                        position: index,
                    });
                }
                self.depth += 1;
                let container = if open == '{' {
                    self.object(index + 1)
                } else {
                    self.array(index + 1)
                };
                self.depth -= 1;
                Some(container?)
            }
            Some('n') => constant("null", Value::Null),
            Some('t') => constant("true", Value::Bool(true)),
            Some('f') => constant("false", Value::Bool(false)),
            Some('N') => constant("NaN", Value::Float(f64::NAN)),
            Some('I') => constant("Infinity", Value::Float(f64::INFINITY)),
            Some('-') if self.at(index + 1) == Some('I') => {
                constant("-Infinity", Value::Float(f64::NEG_INFINITY))
            }
            Some('-' | '0'..='9') => self.number(index)?,
            _ => None,
        })
    }

    fn number(&self, start: usize) -> Result<Scanned, ParseError> {
        let digits_from = |mut index: usize| {
            while self.at(index).is_some_and(|c| c.is_ascii_digit()) {
                index += 1;
            }
            index
        };
        let mut index = start;
        if self.at(index) == Some('-') {
            index += 1;
        }
        let integer_end = match self.at(index) {
            Some('0') => index + 1,
            Some('1'..='9') => digits_from(index + 1),
            _ => return Ok(None),
        };
        let mut end = integer_end;
        let mut is_float = false;
        if self.at(end) == Some('.') && self.at(end + 1).is_some_and(|c| c.is_ascii_digit()) {
            end = digits_from(end + 1);
            is_float = true;
        }
        if matches!(self.at(end), Some('e' | 'E')) {
            let digits = if matches!(self.at(end + 1), Some('+' | '-')) {
                end + 2
            } else {
                end + 1
            };
            if self.at(digits).is_some_and(|c| c.is_ascii_digit()) {
                end = digits_from(digits);
                is_float = true;
            }
        }
        let text: String = self.chars[start..end].iter().collect();
        let value = if is_float {
            // Validated JSON numbers are valid Rust floats, rounded the same way.
            Value::Float(text.parse().unwrap_or(f64::NAN))
        } else {
            let (negative, digits) = match text.strip_prefix('-') {
                Some(digits) => (true, digits),
                None => (false, text.as_str()),
            };
            if digits.len() > INT_MAX_STR_DIGITS {
                return Err(ParseError::IntTooLong);
            }
            Value::Int(normalized_int(negative, digits))
        };
        Ok(Some((value, end)))
    }

    /// The string whose opening quote is just before `start`, and the index after its end quote.
    fn string(&self, start: usize) -> Result<(String, usize), ParseError> {
        let unterminated = || self.syntax("Unterminated string starting at", start - 1);
        let mut text = String::new();
        let mut index = start;
        loop {
            let c = self.at(index).ok_or_else(unterminated)?;
            index += 1;
            match c {
                '"' => return Ok((text, index)),
                '\\' => {}
                c if c < ' ' => return Err(self.syntax("Invalid control character at", index - 1)),
                c => {
                    text.push(c);
                    continue;
                }
            }
            let escape = self.at(index).ok_or_else(unterminated)?;
            index += 1;
            let unescaped = match escape {
                '"' => '"',
                '\\' => '\\',
                '/' => '/',
                'b' => '\u{8}',
                'f' => '\u{c}',
                'n' => '\n',
                'r' => '\r',
                't' => '\t',
                'u' => {
                    let mut code = self.hex4(index)?;
                    index += 4;
                    if (0xd800..0xdc00).contains(&code)
                        && self.at(index) == Some('\\')
                        && self.at(index + 1) == Some('u')
                    {
                        let low = self.hex4(index + 2)?;
                        if (0xdc00..0xe000).contains(&low) {
                            code = 0x10000 + ((code - 0xd800) << 10) + (low - 0xdc00);
                            index += 6;
                        }
                    }
                    char::from_u32(code).ok_or_else(|| {
                        let (line, column) = self.location(start - 1);
                        ParseError::LoneSurrogate {
                            line,
                            column,
                            position: start - 1,
                        }
                    })?
                }
                _ => return Err(self.syntax("Invalid \\escape", index - 2)),
            };
            text.push(unescaped);
        }
    }

    /// The four hex digits at `index`, the one after the `u` of a `\u` escape.
    fn hex4(&self, index: usize) -> Result<u32, ParseError> {
        (index..index + 4)
            .try_fold(0, |code, i| {
                self.at(i)
                    .and_then(|c| c.to_digit(16))
                    .map(|digit| code * 16 + digit)
            })
            .ok_or_else(|| self.syntax("Invalid \\uXXXX escape", index - 1))
    }

    fn object(&mut self, start: usize) -> Result<(Value, usize), ParseError> {
        let mut entries = IndexMap::new();
        let mut index = self.skip_whitespace(start);
        if self.at(index) == Some('}') {
            return Ok((Value::Dict(entries), index + 1));
        }
        loop {
            if self.at(index) != Some('"') {
                return Err(self.syntax("Expecting property name enclosed in double quotes", index));
            }
            let (key, end) = self.string(index + 1)?;
            index = self.skip_whitespace(end);
            if self.at(index) != Some(':') {
                return Err(self.syntax("Expecting ':' delimiter", index));
            }
            index = self.skip_whitespace(index + 1);
            let (value, end) = self
                .value(index)?
                .ok_or_else(|| self.syntax("Expecting value", index))?;
            entries.insert(key, value);
            index = self.skip_whitespace(end);
            match self.at(index) {
                Some('}') => return Ok((Value::Dict(entries), index + 1)),
                Some(',') => {}
                _ => return Err(self.syntax("Expecting ',' delimiter", index)),
            }
            let comma = index;
            index = self.skip_whitespace(index + 1);
            if self.at(index) == Some('}') {
                return Err(self.syntax("Illegal trailing comma before end of object", comma));
            }
        }
    }

    fn array(&mut self, start: usize) -> Result<(Value, usize), ParseError> {
        let mut items = Vec::new();
        let mut index = self.skip_whitespace(start);
        if self.at(index) == Some(']') {
            return Ok((Value::List(items), index + 1));
        }
        loop {
            let (value, end) = self
                .value(index)?
                .ok_or_else(|| self.syntax("Expecting value", index))?;
            items.push(value);
            index = self.skip_whitespace(end);
            match self.at(index) {
                Some(']') => return Ok((Value::List(items), index + 1)),
                Some(',') => {}
                _ => return Err(self.syntax("Expecting ',' delimiter", index)),
            }
            let comma = index;
            index = self.skip_whitespace(index + 1);
            if self.at(index) == Some(']') {
                return Err(self.syntax("Illegal trailing comma before end of array", comma));
            }
        }
    }
}

/// `json.dumps(value, indent=2)`, with its default `ensure_ascii=True`.
pub fn dumps(value: &Value) -> String {
    let mut out = String::new();
    dump(value, 0, &mut out);
    out
}

fn dump(value: &Value, level: usize, out: &mut String) {
    match value {
        Value::Null => out.push_str("null"),
        Value::Bool(flag) => out.push_str(if *flag { "true" } else { "false" }),
        Value::Int(digits) => out.push_str(digits),
        Value::Float(float) => out.push_str(&float_json(*float)),
        Value::Str(text) => dump_str(text, out),
        Value::List(items) => dump_container(('[', ']'), items, level, out, |item, out| {
            dump(item, level + 1, out);
        }),
        Value::Dict(entries) => {
            dump_container(('{', '}'), entries, level, out, |(key, value), out| {
                dump_str(key, out);
                out.push_str(": ");
                dump(value, level + 1, out);
            });
        }
    }
}

fn dump_container<I: IntoIterator>(
    (open, close): (char, char),
    items: I,
    level: usize,
    out: &mut String,
    mut dump_item: impl FnMut(I::Item, &mut String),
) {
    out.push(open);
    let mut empty = true;
    for item in items {
        out.push_str(if empty { "\n" } else { ",\n" });
        empty = false;
        push_indent(level + 1, out);
        dump_item(item, out);
    }
    if !empty {
        out.push('\n');
        push_indent(level, out);
    }
    out.push(close);
}

fn push_indent(level: usize, out: &mut String) {
    out.extend(std::iter::repeat_n("  ", level));
}

/// `float.__repr__`, with the JavaScript names `json` gives to non-finite values.
fn float_json(value: f64) -> String {
    if value.is_nan() {
        "NaN".to_owned()
    } else if value.is_infinite() {
        if value > 0.0 { "Infinity" } else { "-Infinity" }.to_owned()
    } else {
        float_repr(value)
    }
}

/// An ASCII-only JSON string: printable ASCII as is, anything else escaped, astral characters as
/// UTF-16 surrogate pairs.
fn dump_str(text: &str, out: &mut String) {
    out.push('"');
    for c in text.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            '\u{8}' => out.push_str("\\b"),
            '\u{c}' => out.push_str("\\f"),
            ' '..='~' => out.push(c),
            c => {
                for unit in c.encode_utf16(&mut [0; 2]) {
                    // Writing to a String cannot fail.
                    let _ = write!(out, "\\u{unit:04x}");
                }
            }
        }
    }
    out.push('"');
}

#[cfg(test)]
#[expect(clippy::unwrap_used, reason = "tests fail loudly on bad fixtures")]
mod tests {
    use super::*;

    fn dict(entries: &[(&str, Value)]) -> Value {
        Value::Dict(
            entries
                .iter()
                .map(|(k, v)| ((*k).to_owned(), v.clone()))
                .collect(),
        )
    }

    fn syntax_error(text: &str) -> String {
        parse(text).map_or_else(
            |error| error.to_string(),
            |value| format!("parsed {value:?}"),
        )
    }

    #[test]
    fn dumps_indents_like_python() {
        let value = dict(&[
            ("a", Value::List(vec![Value::Int("1".into()), Value::Null])),
            ("b", dict(&[])),
            ("c", Value::List(vec![])),
            ("d", dict(&[("e", Value::Bool(true))])),
        ]);
        assert_eq!(
            dumps(&value),
            "{\n  \"a\": [\n    1,\n    null\n  ],\n  \"b\": {},\n  \"c\": [],\n  \"d\": {\n    \"e\": true\n  }\n}"
        );
    }

    #[test]
    fn dumps_floats_like_python() {
        let floats = [
            1.0,
            1e-5,
            1e16,
            -0.0,
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
        ];
        let dumped: Vec<String> = floats.iter().map(|&f| dumps(&Value::Float(f))).collect();
        assert_eq!(
            dumped,
            [
                "1.0",
                "1e-05",
                "1e+16",
                "-0.0",
                "NaN",
                "Infinity",
                "-Infinity"
            ]
        );
    }

    #[test]
    fn dumps_escapes_everything_but_printable_ascii() {
        let text = "\"\\/\n\r\t\u{8}\u{c}\u{1}\u{7f}é😀~";
        assert_eq!(
            dumps(&Value::Str(text.to_owned())),
            r#""\"\\/\n\r\t\b\f\u0001\u007f\u00e9\ud83d\ude00~""#
        );
    }

    #[test]
    fn parse_reads_numbers_as_python_does() {
        let parsed = parse("[1.50, 1e3, 1e400, -0.0, -0, 0, 123456789012345678901234567890]");
        let expected = Value::List(vec![
            Value::Float(1.5),
            Value::Float(1000.0),
            Value::Float(f64::INFINITY),
            Value::Float(-0.0),
            Value::Int("0".into()),
            Value::Int("0".into()),
            Value::Int("123456789012345678901234567890".into()),
        ]);
        assert_eq!(parsed, Ok(expected));
    }

    #[test]
    fn parse_keeps_first_place_and_last_value_of_repeated_keys() {
        let parsed = parse(r#"{"a": 1, "b": 2, "a": 3}"#);
        assert_eq!(
            parsed,
            Ok(dict(&[
                ("a", Value::Int("3".into())),
                ("b", Value::Int("2".into()))
            ]))
        );
    }

    #[test]
    fn parse_accepts_python_constants_and_escapes() {
        let parsed = parse(r#"[NaN, Infinity, -Infinity, "\ud83d\ude00\/\u00E9"]"#);
        let Value::List(items) = parsed.unwrap() else {
            unreachable!("not a list");
        };
        assert!(matches!(items[0], Value::Float(f) if f.is_nan()));
        assert_eq!(
            items[1..],
            [
                Value::Float(f64::INFINITY),
                Value::Float(f64::NEG_INFINITY),
                Value::Str("😀/é".into())
            ]
        );
    }

    #[test]
    fn parse_errors_read_as_python_s() {
        let cases = [
            ("", "Expecting value: line 1 column 1 (char 0)"),
            (
                "{",
                "Expecting property name enclosed in double quotes: line 1 column 2 (char 1)",
            ),
            (
                "{\"a\" 1}",
                "Expecting ':' delimiter: line 1 column 6 (char 5)",
            ),
            (
                "{\"a\":1,}",
                "Illegal trailing comma before end of object: line 1 column 7 (char 6)",
            ),
            ("[1 2]", "Expecting ',' delimiter: line 1 column 4 (char 3)"),
            (
                "[1,\n  2,\n x]",
                "Expecting value: line 3 column 2 (char 10)",
            ),
            ("\"a\\x\"", "Invalid \\escape: line 1 column 3 (char 2)"),
            (
                "\"a\\u12G4\"",
                "Invalid \\uXXXX escape: line 1 column 4 (char 3)",
            ),
            (
                "\"a\u{1}\"",
                "Invalid control character at: line 1 column 3 (char 2)",
            ),
            (
                "\"abc",
                "Unterminated string starting at: line 1 column 1 (char 0)",
            ),
            ("[1.]", "Expecting ',' delimiter: line 1 column 3 (char 2)"),
            ("{}x", "Extra data: line 1 column 3 (char 2)"),
            (
                "\u{feff}{}",
                "Unexpected UTF-8 BOM (decode using utf-8-sig): line 1 column 1 (char 0)",
            ),
        ];
        for (text, message) in cases {
            assert_eq!(syntax_error(text), message, "{text:?}");
        }
    }

    #[test]
    fn parse_counts_a_carriage_return_as_a_newline() {
        assert_eq!(
            syntax_error("[\r\n\r x]"),
            "Expecting value: line 3 column 2 (char 4)"
        );
    }

    #[test]
    fn parse_refuses_unpaired_surrogates() {
        assert!(matches!(
            parse(r#""\ud800""#),
            Err(ParseError::LoneSurrogate { .. })
        ));
    }

    #[test]
    fn parse_refuses_ints_longer_than_python_converts() {
        assert_eq!(parse(&"1".repeat(4301)), Err(ParseError::IntTooLong));
    }
}
