//! Reads the `config` and `help_text` dicts of `scripts/valhalla_build_config` and writes the
//! Rust table of their leaves. Only the Python literal subset those dicts use is understood;
//! anything else is an error, which fails the build.
use std::error::Error;
use std::fmt::Write as _;

pub type Result<T> = std::result::Result<T, Box<dyn Error>>;

/// Options of the script that are not config leaves.
const FIXED_OPTIONS: [&str; 5] = ["help", "merge-config", "output", "remove-old", "report"];

/// The options the script adds by name, all but argparse's own `-h/--help`.
const NAMED_OPTIONS: [&str; 8] = [
    "-m",
    "--merge-config",
    "-o",
    "--output",
    "-r",
    "--remove-old",
    "-p",
    "--report",
];

/// Top-level blocks of the script whose behaviour the Rust port reproduces: the line a block
/// starts with, the line it ends before (by default the next unindented line other than a
/// closing bracket) and the FNV-1a hash of its lines with trailing whitespace trimmed.
const GUARDED_BLOCKS: [(&str, Option<&str>, u64); 11] = [
    ("class MergeReport:", None, 0xd1d89beeffa36c5a),
    ("def should_remove_key(", None, 0x675f110d479d5a98),
    ("def merge_json(", None, 0x0e77b583639ed161),
    ("def strtobool(", None, 0x58d965ebb6746cc1),
    ("class Optional:", None, 0xabfb199bc2ba8f93),
    ("def ipc_endpoint(", None, 0x22f1d8952d64dd36),
    ("def add_leaf_args(", None, 0x03c147502aaad2d9),
    ("def override_config(", None, 0xcc54159548a6ecc5),
    ("leaves = []", Some("def main("), 0x9bead09274d7ec70),
    ("def main(", None, 0x6cd64d5e372eef27),
    ("if __name__ ==", None, 0x9b2c1201677e498a),
];

/// Fails when the script changed in a way the generated table cannot follow, or the Rust port
/// may no longer reproduce.
pub fn check_drift(source: &str) -> Result<()> {
    let lines: Vec<&str> = source.lines().map(str::trim_end).collect();
    let mut problems = Vec::new();
    for line in &lines {
        let mutation = [
            "config[",
            "config.",
            "help_text[",
            "help_text.",
            "del config",
            "del help_text",
        ]
        .iter()
        .find(|prefix| line.starts_with(*prefix));
        if let Some(prefix) = mutation {
            problems.push(format!(
                "top-level `{prefix}` changes a dict the table is read from"
            ));
        }
    }
    let mut named = named_options(source);
    named.sort_unstable();
    let mut expected = NAMED_OPTIONS.to_vec();
    expected.sort_unstable();
    if named != expected {
        problems.push(format!(
            "add_argument() option strings are {named:?}, expected {expected:?}"
        ));
    }
    for (start, end, expected) in GUARDED_BLOCKS {
        match block(&lines, start, end) {
            None => problems.push(format!("no `{start}` block")),
            Some(text) => {
                let hash = fnv1a(&text);
                if hash != expected {
                    problems.push(format!(
                        "`{start}` changed: check that src/build_config still reproduces it, then \
                         set its hash in build/py_literal.rs to {hash:#018x}"
                    ));
                }
            }
        }
    }
    if problems.is_empty() {
        Ok(())
    } else {
        Err(problems.join("\n").into())
    }
}

/// The lines from the one starting with `start` to the one before `end`, or before the next
/// unindented line other than a closing bracket.
fn block(lines: &[&str], start: &str, end: Option<&str>) -> Option<String> {
    let first = lines.iter().position(|line| line.starts_with(start))?;
    let length = lines[first + 1..]
        .iter()
        .position(|line| match end {
            Some(end) => line.starts_with(end),
            None => !line.is_empty() && !line.starts_with([' ', '\t', ')', ']', '}']),
        })
        .unwrap_or(lines.len() - first - 1);
    Some(lines[first..=first + length].join("\n"))
}

/// The string literals starting with `-` in the script's `add_argument(` calls.
fn named_options(source: &str) -> Vec<String> {
    let mut options = Vec::new();
    for (index, _) in source.match_indices("add_argument(") {
        let mut depth = 0;
        let mut chars = source[index + "add_argument".len()..].chars();
        while let Some(c) = chars.next() {
            match c {
                '(' => depth += 1,
                ')' if depth == 1 => break,
                ')' => depth -= 1,
                quote @ ('"' | '\'') => {
                    let text: String = chars.by_ref().take_while(|&c| c != quote).collect();
                    if text.starts_with('-') {
                        options.push(text);
                    }
                }
                _ => {}
            }
        }
    }
    options
}

fn fnv1a(text: &str) -> u64 {
    text.bytes().fold(0xcbf2_9ce4_8422_2325, |hash, byte| {
        (hash ^ u64::from(byte)).wrapping_mul(0x0100_0000_01b3)
    })
}

#[derive(Debug)]
enum Py {
    Dict(Vec<(String, Py)>),
    List(Vec<Py>),
    Str(String),
    /// Decimal digits with an optional leading `-`, without leading zeros.
    Int(String),
    Float(f64),
    Bool(bool),
    None,
    Optional(&'static str),
    Endpoint {
        ipc: String,
        port: String,
    },
}

/// The Rust source of `LEAVES` for the script text `source`, once [`check_drift`] passed.
pub fn generate(source: &str) -> Result<String> {
    check_drift(source)?;
    table(source)
}

fn table(source: &str) -> Result<String> {
    let config = parse_assignment(source, "config")?;
    let help = parse_assignment(source, "help_text")?;
    let mut leaves = Vec::new();
    collect_leaves(&config, &mut Vec::new(), &mut leaves)?;
    let mut options: Vec<String> = FIXED_OPTIONS.iter().map(|o| (*o).to_owned()).collect();
    let mut code = String::from(
        "/// Every leaf, in the order of the script's `config` dict.\npub static LEAVES: &[Leaf] = &[\n",
    );
    for (path, value) in &leaves {
        let option = path.join("-").replace('_', "-");
        if options.contains(&option) {
            return Err(format!("option --{option} is defined twice").into());
        }
        options.push(option.clone());
        let help = lookup_help(&help, path)?;
        let default = match value {
            Py::Optional(kind) => format!("DefaultValue::Optional(OptionalKind::{kind})"),
            value => format!("DefaultValue::Value({})", literal(value)?),
        };
        writeln!(
            code,
            "    Leaf {{ path: &{path:?}, option: {option:?}, default: {default}, help: {help:?} }},"
        )?;
    }
    code.push_str("];\n");
    Ok(code)
}

/// The leaves of the config tree in dict order, the way `add_leaf_args` walks it.
fn collect_leaves<'a>(
    node: &'a Py,
    path: &mut Vec<&'a str>,
    leaves: &mut Vec<(Vec<&'a str>, &'a Py)>,
) -> Result<()> {
    match node {
        Py::Dict(entries) => {
            if entries.is_empty() {
                return Err(format!("empty dict at '{}' is not supported", path.join(".")).into());
            }
            for (key, value) in entries {
                if key.is_empty() || !key.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
                    return Err(format!("config key {key:?} would not map to an option").into());
                }
                path.push(key);
                collect_leaves(value, path, leaves)?;
                path.pop();
            }
        }
        _ => leaves.push((path.clone(), node)),
    }
    Ok(())
}

fn lookup_help<'a>(help: &'a Py, path: &[&str]) -> Result<&'a str> {
    let mut node = help;
    for key in path {
        let Py::Dict(entries) = node else {
            return Err(format!("help_text has no dict above '{}'", path.join(".")).into());
        };
        node = entries
            .iter()
            .find_map(|(k, v)| (k == key).then_some(v))
            .ok_or_else(|| format!("help_text has no entry for '{}'", path.join(".")))?;
    }
    match node {
        Py::Str(text) => Ok(text),
        _ => Err(format!("help_text entry for '{}' is not a string", path.join(".")).into()),
    }
}

/// The Rust expression of a default value, as a `Literal`.
fn literal(value: &Py) -> Result<String> {
    Ok(match value {
        Py::Str(text) => format!("Literal::Str({text:?})"),
        Py::Int(digits) => format!("Literal::Int({digits:?})"),
        // From bits: Rust's float literal syntax has no inf.
        Py::Float(float) => format!("Literal::Float(f64::from_bits({:#x}))", float.to_bits()),
        Py::Bool(flag) => format!("Literal::Bool({flag})"),
        Py::None => "Literal::Null".to_owned(),
        Py::Endpoint { ipc, port } => {
            format!("Literal::Endpoint {{ ipc: {ipc:?}, port: {port:?} }}")
        }
        Py::List(items) => {
            let items: Vec<String> = items.iter().map(literal).collect::<Result<_>>()?;
            format!("Literal::List(&[{}])", items.join(", "))
        }
        Py::Dict(_) | Py::Optional(_) => {
            return Err(format!("unsupported value inside a list: {value:?}").into());
        }
    })
}

/// Parses the dict literal assigned to `name` at the top level of the script, which must assign
/// it once. Lines may end in `\r\n`.
fn parse_assignment(source: &str, name: &str) -> Result<Py> {
    let prefix = format!("{name} = ");
    let mut starts = source
        .match_indices(&prefix)
        .filter(|&(index, _)| index == 0 || source[..index].ends_with('\n'))
        .map(|(index, _)| index + prefix.len());
    let start = starts
        .next()
        .ok_or_else(|| format!("no top-level '{name} = '"))?;
    if starts.next().is_some() {
        return Err(format!("'{name}' is assigned more than once").into());
    }
    let mut parser = Parser {
        text: source,
        pos: start,
    };
    let value = parser.value()?;
    if !matches!(value, Py::Dict(_)) {
        return Err(format!("'{name}' is not a dict").into());
    }
    Ok(value)
}

struct Parser<'a> {
    text: &'a str,
    pos: usize,
}

impl<'a> Parser<'a> {
    fn error(&self, message: &str) -> Box<dyn Error> {
        let line = self.text[..self.pos].matches('\n').count() + 1;
        format!("line {line}: {message}").into()
    }

    fn rest(&self) -> &'a str {
        &self.text[self.pos..]
    }

    /// Skips whitespace, newlines included as everything parsed sits inside brackets, and comments.
    fn skip_blank(&mut self) {
        loop {
            let rest = self.rest();
            let trimmed = rest.trim_start();
            self.pos += rest.len() - trimmed.len();
            if trimmed.starts_with('#') {
                self.pos += trimmed.find('\n').unwrap_or(trimmed.len());
            } else {
                return;
            }
        }
    }

    fn peek(&mut self) -> Option<char> {
        self.skip_blank();
        self.rest().chars().next()
    }

    fn eat(&mut self, expected: char) -> bool {
        if self.peek() == Some(expected) {
            self.pos += expected.len_utf8();
            true
        } else {
            false
        }
    }

    fn expect(&mut self, expected: char) -> Result<()> {
        if self.eat(expected) {
            Ok(())
        } else {
            Err(self.error(&format!("expected '{expected}'")))
        }
    }

    fn value(&mut self) -> Result<Py> {
        match self.peek() {
            Some('{') => self.dict(),
            Some('[') => self.list(),
            Some('(') => {
                self.expect('(')?;
                let value = self.value()?;
                if !self.eat(')') {
                    return Err(self.error("expected ')': tuples are not supported"));
                }
                Ok(value)
            }
            Some('"' | '\'') => self.strings(),
            Some('-' | '+' | '.' | '0'..='9') => self.number(),
            Some(c) if c.is_ascii_alphabetic() || c == '_' => self.name(),
            _ => Err(self.error("unsupported expression")),
        }
    }

    fn dict(&mut self) -> Result<Py> {
        self.expect('{')?;
        let mut entries: Vec<(String, Py)> = Vec::new();
        while !self.eat('}') {
            let Py::Str(key) = self.value()? else {
                return Err(self.error("dict keys must be strings"));
            };
            if entries.iter().any(|(k, _)| *k == key) {
                return Err(self.error(&format!("duplicate key {key:?}")));
            }
            self.expect(':')?;
            let value = self.value()?;
            entries.push((key, value));
            if !self.eat(',') {
                self.expect('}')?;
                break;
            }
        }
        Ok(Py::Dict(entries))
    }

    fn list(&mut self) -> Result<Py> {
        self.expect('[')?;
        let mut items = Vec::new();
        while !self.eat(']') {
            items.push(self.value()?);
            if !self.eat(',') {
                self.expect(']')?;
                break;
            }
        }
        Ok(Py::List(items))
    }

    /// One string literal or several adjacent ones, which Python concatenates.
    fn strings(&mut self) -> Result<Py> {
        let mut text = String::new();
        while let Some(quote @ ('"' | '\'')) = self.peek() {
            if self.rest().starts_with(&quote.to_string().repeat(3)) {
                return Err(self.error("triple-quoted strings are not supported"));
            }
            self.pos += 1;
            let mut chars = self.rest().char_indices();
            loop {
                let Some((index, c)) = chars.next() else {
                    return Err(self.error("unterminated string"));
                };
                match c {
                    '\n' => return Err(self.error("unterminated string")),
                    c if c == quote => {
                        self.pos += index + 1;
                        break;
                    }
                    '\\' => match chars.next().map(|(_, e)| e) {
                        Some(e @ ('\\' | '\'' | '"')) => text.push(e),
                        Some('n') => text.push('\n'),
                        Some('t') => text.push('\t'),
                        Some('r') => text.push('\r'),
                        _ => return Err(self.error("unsupported string escape")),
                    },
                    c => text.push(c),
                }
            }
        }
        Ok(Py::Str(text))
    }

    /// A Python int or float literal: digits with single underscores between them, then for a
    /// float a `.` fraction and/or an exponent.
    fn number(&mut self) -> Result<Py> {
        let negative = if self.eat('-') {
            true
        } else {
            self.eat('+');
            false
        };
        self.skip_blank();
        let rest = self.rest();
        let bytes = rest.as_bytes();
        let digit = |index: usize| bytes.get(index).is_some_and(u8::is_ascii_digit);
        let digits_from = |start: usize| {
            let mut end = start;
            while digit(end) || (bytes.get(end) == Some(&b'_') && end > start && digit(end + 1)) {
                end += 1;
            }
            end
        };
        let whole_end = digits_from(0);
        let mut end = whole_end;
        let mut float = false;
        if bytes.get(end) == Some(&b'.') {
            float = true;
            end = digits_from(end + 1);
        }
        let has_digits = whole_end > 0 || end > whole_end + 1;
        if has_digits && matches!(bytes.get(end), Some(b'e' | b'E')) {
            let sign = usize::from(matches!(bytes.get(end + 1), Some(b'+' | b'-')));
            let exponent_end = digits_from(end + 1 + sign);
            if exponent_end > end + 1 + sign {
                float = true;
                end = exponent_end;
            }
        }
        let trailing = rest[end..]
            .find(|c: char| !(c.is_ascii_alphanumeric() || c == '_' || c == '.'))
            .unwrap_or(rest.len() - end);
        let token = &rest[..end + trailing];
        self.pos += token.len();
        let invalid = || self.error(&format!("unsupported number literal {token:?}"));
        let digits = token.replace('_', "");
        // Python refuses leading zeros in a nonzero int, such as `01`.
        let leading_zero = digits.len() > 1
            && digits.starts_with('0')
            && digits.contains(['1', '2', '3', '4', '5', '6', '7', '8', '9']);
        if !has_digits || trailing > 0 || (!float && leading_zero) {
            return Err(invalid());
        }
        if float {
            let value: f64 = digits.parse().map_err(|_| invalid())?;
            return Ok(Py::Float(if negative { -value } else { value }));
        }
        let digits = digits.trim_start_matches('0');
        Ok(Py::Int(match (digits.is_empty(), negative) {
            (true, _) => "0".to_owned(),
            (false, true) => format!("-{digits}"),
            (false, false) => digits.to_owned(),
        }))
    }

    fn name(&mut self) -> Result<Py> {
        let rest = self.rest();
        let length = rest
            .find(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
            .unwrap_or(rest.len());
        let name = &rest[..length];
        self.pos += length;
        match name {
            "True" => Ok(Py::Bool(true)),
            "False" => Ok(Py::Bool(false)),
            "None" => Ok(Py::None),
            "Optional" => {
                self.expect('(')?;
                self.skip_blank();
                let rest = self.rest();
                let length = rest
                    .find(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
                    .unwrap_or(rest.len());
                let kind = match &rest[..length] {
                    "int" => "Int",
                    "str" => "Str",
                    "float" => "Float",
                    "bool" => "Bool",
                    "list" => "List",
                    _ => {
                        return Err(
                            self.error("Optional() takes one of int, str, float, bool, list")
                        );
                    }
                };
                self.pos += length;
                self.expect(')')?;
                Ok(Py::Optional(kind))
            }
            "ipc_endpoint" => {
                self.expect('(')?;
                let Py::Str(ipc) = self.value()? else {
                    return Err(self.error("ipc_endpoint() takes a string path"));
                };
                self.expect(',')?;
                let Py::Int(port) = self.value()? else {
                    return Err(self.error("ipc_endpoint() takes an int port"));
                };
                self.eat(',');
                self.expect(')')?;
                Ok(Py::Endpoint { ipc, port })
            }
            _ => Err(self.error(&format!("unsupported name '{name}'"))),
        }
    }
}

#[cfg(test)]
#[expect(clippy::unwrap_used, reason = "tests fail loudly on bad fixtures")]
mod tests {
    use super::*;

    fn script(config: &str, help: &str) -> String {
        format!("import sys\n\nconfig = {config}\n\nhelp_text = {help}\n")
    }

    fn error(config: &str, help: &str) -> String {
        table(&script(config, help)).unwrap_err().to_string()
    }

    #[test]
    fn every_supported_literal_becomes_a_leaf() {
        let table = table(&script(
            r#"{
    "a": {"s": 's\'"', "i": -1_000, "f": 1e8, "t": True, "n": None, "l": [7, "x", [.5]],
          "o": Optional(list), "e": ipc_endpoint("/tmp/x", 8003),},  # comment
}"#,
            r#"{"a": {"__note__": ("ignored " "note"), "s": ("con" 'cat'), "i": "i", "f": "f",
    "t": "t", "n": "n", "l": "l", "o": "o", "e": "e"}}"#,
        ))
        .unwrap();
        for line in [
            r#"Leaf { path: &["a", "s"], option: "a-s", default: DefaultValue::Value(Literal::Str("s'\"")), help: "concat" },"#,
            r#"default: DefaultValue::Value(Literal::Int("-1000")),"#,
            "default: DefaultValue::Value(Literal::Float(f64::from_bits(0x4197d78400000000))),",
            "default: DefaultValue::Value(Literal::Bool(true)),",
            "default: DefaultValue::Value(Literal::Null),",
            r#"Literal::List(&[Literal::Int("7"), Literal::Str("x"), Literal::List(&[Literal::Float(f64::from_bits(0x3fe0000000000000))])])"#,
            "default: DefaultValue::Optional(OptionalKind::List),",
            r#"Literal::Endpoint { ipc: "/tmp/x", port: "8003" }"#,
        ] {
            assert!(table.contains(line), "{line} not in\n{table}");
        }
    }

    #[test]
    fn a_missing_assignment_is_an_error() {
        assert_eq!(
            table("config = {}\n").unwrap_err().to_string(),
            "no top-level 'help_text = '"
        );
    }

    #[test]
    fn a_leaf_without_help_is_an_error() {
        assert_eq!(error(r#"{"a": 1}"#, "{}"), "help_text has no entry for 'a'");
    }

    #[test]
    fn a_help_that_is_not_a_string_is_an_error() {
        assert_eq!(
            error(r#"{"a": 1}"#, r#"{"a": 1}"#),
            "help_text entry for 'a' is not a string"
        );
    }

    #[test]
    fn unsupported_constructs_are_errors() {
        for (config, message) in [
            (r#"{"a": os.sep}"#, "line 3: unsupported name 'os'"),
            (r#"{"a": f"x"}"#, "line 3: unsupported name 'f'"),
            (
                r#"{"a": ("x", "y")}"#,
                "line 3: expected ')': tuples are not supported",
            ),
            (
                r#"{"a": """x"""}"#,
                "line 3: triple-quoted strings are not supported",
            ),
            (r#"{"a": "\x41"}"#, "line 3: unsupported string escape"),
            (r#"{"a": 01}"#, "line 3: unsupported number literal \"01\""),
            (
                r#"{"a": 0x1}"#,
                "line 3: unsupported number literal \"0x1\"",
            ),
            (
                r#"{"a": Optional(dict)}"#,
                "line 3: Optional() takes one of int, str, float, bool, list",
            ),
            (
                r#"{"a": ipc_endpoint("/x", "1")}"#,
                "line 3: ipc_endpoint() takes an int port",
            ),
            (
                r#"{"a": [{"b": 1}]}"#,
                "unsupported value inside a list: Dict([(\"b\", Int(\"1\"))])",
            ),
            (r#"{1: 2}"#, "line 3: dict keys must be strings"),
            (r#"{"a": 1, "a": 2}"#, "line 3: duplicate key \"a\""),
            (r#"{"a": {}}"#, "empty dict at 'a' is not supported"),
            (
                r#"{"a-b": 1}"#,
                "config key \"a-b\" would not map to an option",
            ),
            (r#"{"a": 1"#, "line 5: expected '}'"),
        ] {
            assert_eq!(error(config, r#"{"a": "h"}"#), message, "{config}");
        }
    }

    #[test]
    fn options_that_collide_are_errors() {
        assert_eq!(
            error(
                r#"{"a_b": 1, "a": {"b": 2}}"#,
                r#"{"a_b": "h", "a": {"b": "h"}}"#
            ),
            "option --a-b is defined twice"
        );
        assert_eq!(
            error(
                r#"{"merge": {"config": 1}}"#,
                r#"{"merge": {"config": "h"}}"#
            ),
            "option --merge-config is defined twice"
        );
    }

    const SCRIPT: &str = include_str!("../../../../../../scripts/valhalla_build_config");

    fn drift(source: &str) -> String {
        generate(source).unwrap_err().to_string()
    }

    #[test]
    fn the_script_generates_a_table() {
        assert!(
            generate(SCRIPT)
                .unwrap()
                .contains("option: \"statsd-tags\"")
        );
    }

    #[test]
    fn a_script_with_crlf_line_ends_generates_the_same_table() {
        let crlf = SCRIPT.replace('\n', "\r\n");
        assert_eq!(generate(&crlf).unwrap(), generate(SCRIPT).unwrap());
    }

    #[test]
    fn a_second_assignment_is_an_error() {
        let source = SCRIPT.replace("def add_leaf_args(", "config = {}\n\n\ndef add_leaf_args(");
        assert_eq!(drift(&source), "'config' is assigned more than once");
    }

    #[test]
    fn a_change_to_a_guarded_function_is_an_error() {
        let source = SCRIPT.replace(r#""y", "yes""#, r#""y", "yes", "yup""#);
        assert!(
            drift(&source).starts_with("`def strtobool(` changed: "),
            "{}",
            drift(&source)
        );
    }

    #[test]
    fn a_missing_guarded_function_is_an_error() {
        let source = SCRIPT.replace("def ipc_endpoint(", "def endpoint(");
        assert!(drift(&source).contains("no `def ipc_endpoint(` block"));
    }

    #[test]
    fn top_level_changes_to_the_dicts_are_errors() {
        for line in [
            "config[\"x\"] = 1",
            "help_text.update({})",
            "del config[\"x\"]",
        ] {
            let source = format!("{SCRIPT}\n{line}\n");
            assert!(
                drift(&source).contains("changes a dict the table is read from"),
                "{line}"
            );
        }
    }

    #[test]
    fn another_named_option_is_an_error() {
        let source = format!("{SCRIPT}\nparser.add_argument(\"-x\", \"--extra\")\n");
        assert!(drift(&source).starts_with("add_argument() option strings are "));
    }
}
