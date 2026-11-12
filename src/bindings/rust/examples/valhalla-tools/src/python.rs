//! Python's text representations, for messages that must read as the scripts' do.

use crate::python_printable::is_printable;

/// `repr()` of a Python float: the shortest round-tripping digits, in positional notation
/// for exponents from -4 to 15 and in scientific notation, such as `1e-05`, otherwise.
pub(crate) fn float_repr(value: f64) -> String {
    if value.is_nan() {
        return "nan".to_owned();
    }
    if value.is_infinite() {
        return if value > 0.0 { "inf" } else { "-inf" }.to_owned();
    }
    let scientific = format!("{value:e}");
    let exponent = scientific
        .split_once('e')
        .and_then(|(_, exponent)| exponent.parse::<i32>().ok());
    match exponent {
        Some(exponent) if !(-4..16).contains(&exponent) => {
            let mantissa = scientific
                .split_once('e')
                .map_or("", |(mantissa, _)| mantissa);
            let sign = if exponent < 0 { '-' } else { '+' };
            format!("{mantissa}e{sign}{:02}", exponent.unsigned_abs())
        }
        _ => {
            let positional = value.to_string();
            if positional.contains('.') {
                positional
            } else {
                positional + ".0"
            }
        }
    }
}

/// `repr()` of a Python list of floats.
pub(crate) fn float_list_repr(values: &[f64]) -> String {
    let reprs: Vec<_> = values.iter().copied().map(float_repr).collect();
    format!("[{}]", reprs.join(", "))
}

/// Python's limit on the digits of an int converted from a string (`sys.get_int_max_str_digits`).
pub(crate) const INT_MAX_STR_DIGITS: usize = 4300;

/// First code point of each run of ten Unicode decimal digits (category Nd) of Python 3.14's
/// Unicode 16.0, which `int()` and `float()` accept like ASCII digits.
const DECIMAL_ZEROS: [u32; 76] = [
    0x30, 0x660, 0x6F0, 0x7C0, 0x966, 0x9E6, 0xA66, 0xAE6, 0xB66, 0xBE6, 0xC66, 0xCE6, 0xD66,
    0xDE6, 0xE50, 0xED0, 0xF20, 0x1040, 0x1090, 0x17E0, 0x1810, 0x1946, 0x19D0, 0x1A80, 0x1A90,
    0x1B50, 0x1BB0, 0x1C40, 0x1C50, 0xA620, 0xA8D0, 0xA900, 0xA9D0, 0xA9F0, 0xAA50, 0xABF0, 0xFF10,
    0x104A0, 0x10D30, 0x10D40, 0x11066, 0x110F0, 0x11136, 0x111D0, 0x112F0, 0x11450, 0x114D0,
    0x11650, 0x116C0, 0x116D0, 0x116DA, 0x11730, 0x118E0, 0x11950, 0x11BF0, 0x11C50, 0x11D50,
    0x11DA0, 0x11F50, 0x16130, 0x16A60, 0x16AC0, 0x16B50, 0x16D70, 0x1CCF0, 0x1D7CE, 0x1D7D8,
    0x1D7E2, 0x1D7EC, 0x1D7F6, 0x1E140, 0x1E2F0, 0x1E4F0, 0x1E5F1, 0x1E950, 0x1FBF0,
];

/// Whether `c` is a Unicode decimal digit, which `\d` matches in Python's `re`.
pub(crate) fn is_decimal_digit(c: char) -> bool {
    decimal_value(c).is_some()
}

fn decimal_value(c: char) -> Option<u8> {
    let code = u32::from(c);
    let zero = DECIMAL_ZEROS
        .partition_point(|&zero| zero <= code)
        .checked_sub(1)?;
    let digit = code - DECIMAL_ZEROS[zero];
    u8::try_from(digit).ok().filter(|&digit| digit < 10)
}

/// The non-ASCII characters of `str.isspace()`.
fn is_unicode_space(c: char) -> bool {
    matches!(
        c,
        '\u{85}' | '\u{a0}' | '\u{1680}' | '\u{2000}'
            ..='\u{200a}' | '\u{2028}' | '\u{2029}' | '\u{202f}' | '\u{205f}' | '\u{3000}'
    )
}

/// CPython's first step of `int()` and `float()`: ASCII is kept, Unicode spaces become `' '` and
/// Unicode decimal digits their ASCII digit; any other character makes the text invalid.
fn to_ascii_number(text: &str) -> Option<String> {
    text.chars()
        .map(|c| {
            if c.is_ascii() {
                Some(c)
            } else if is_unicode_space(c) {
                Some(' ')
            } else {
                decimal_value(c).map(|digit| char::from(b'0' + digit))
            }
        })
        .collect()
}

/// Whether `text` is digits with single underscores between them, as Python's literals allow.
fn is_underscored_digits(text: &str) -> bool {
    !text.is_empty()
        && !text.starts_with('_')
        && !text.ends_with('_')
        && !text.contains("__")
        && text.bytes().all(|b| b.is_ascii_digit() || b == b'_')
}

/// Python's `int(text)`: the normalized decimal digits of the value, with a leading `-` when
/// negative, or `None` where Python raises `ValueError`.
pub(crate) fn parse_int(text: &str) -> Option<String> {
    let ascii = to_ascii_number(text)?;
    let trimmed = ascii.trim_matches(|c: char| c.is_ascii_whitespace() || c == '\u{b}');
    let (negative, body) = match trimmed.strip_prefix('-') {
        Some(body) => (true, body),
        None => (false, trimmed.strip_prefix('+').unwrap_or(trimmed)),
    };
    if !is_underscored_digits(body) {
        return None;
    }
    let digits = body.replace('_', "");
    if digits.len() > INT_MAX_STR_DIGITS {
        return None;
    }
    Some(normalized_int(negative, &digits))
}

/// Decimal digits without leading zeros, and no sign on zero.
pub(crate) fn normalized_int(negative: bool, digits: &str) -> String {
    let digits = digits.trim_start_matches('0');
    match (digits.is_empty(), negative) {
        (true, _) => "0".to_owned(),
        (false, true) => format!("-{digits}"),
        (false, false) => digits.to_owned(),
    }
}

/// Python's `float(text)`, or `None` where Python raises `ValueError`.
pub(crate) fn parse_float(text: &str) -> Option<f64> {
    let ascii = to_ascii_number(text)?;
    // Underscores must sit between two digits of the text as given, spaces included.
    let bytes = ascii.as_bytes();
    let underscores_ok = bytes.iter().enumerate().all(|(index, &b)| {
        b != b'_'
            || (index > 0
                && bytes[index - 1].is_ascii_digit()
                && bytes.get(index + 1).is_some_and(u8::is_ascii_digit))
    });
    if !underscores_ok {
        return None;
    }
    let cleaned = ascii.replace('_', "");
    let trimmed = cleaned.trim_matches(|c: char| c.is_ascii_whitespace() || c == '\u{b}');
    let (negative, body) = match trimmed.strip_prefix('-') {
        Some(body) => (true, body),
        None => (false, trimmed.strip_prefix('+').unwrap_or(trimmed)),
    };
    let magnitude = match body.to_ascii_lowercase().as_str() {
        "inf" | "infinity" => f64::INFINITY,
        "nan" => f64::NAN,
        _ if is_decimal_float(body) => body.parse().ok()?,
        _ => return None,
    };
    Some(if negative { -magnitude } else { magnitude })
}

/// `digits [. digits] [e [sign] digits]`, with at least one mantissa digit.
fn is_decimal_float(text: &str) -> bool {
    let (mantissa, exponent) = match text.find(['e', 'E']) {
        Some(index) => (&text[..index], Some(&text[index + 1..])),
        None => (text, None),
    };
    let (whole, fraction) = mantissa.split_once('.').unwrap_or((mantissa, ""));
    let all_digits = |part: &str| part.bytes().all(|b| b.is_ascii_digit());
    let exponent_ok = exponent.is_none_or(|exponent| {
        let digits = exponent.strip_prefix(['+', '-']).unwrap_or(exponent);
        !digits.is_empty() && all_digits(digits)
    });
    all_digits(whole)
        && all_digits(fraction)
        && !(whole.is_empty() && fraction.is_empty())
        && exponent_ok
}

/// The script's `strtobool`.
pub(crate) fn strtobool(text: &str) -> Option<bool> {
    match text.to_lowercase().as_str() {
        "y" | "yes" | "t" | "true" | "on" | "1" => Some(true),
        "n" | "no" | "f" | "false" | "off" | "0" => Some(false),
        _ => None,
    }
}

/// `repr()` of a Python str: single quotes unless only double quotes avoid escaping, and
/// escapes for the quote, backslashes and characters that are not printable.
pub(crate) fn str_repr(text: &str) -> String {
    let quote = if text.contains('\'') && !text.contains('"') {
        '"'
    } else {
        '\''
    };
    let mut repr = String::from(quote);
    for c in text.chars() {
        match c {
            c if c == quote || c == '\\' => {
                repr.push('\\');
                repr.push(c);
            }
            '\t' => repr.push_str("\\t"),
            '\n' => repr.push_str("\\n"),
            '\r' => repr.push_str("\\r"),
            ' '..='~' => repr.push(c),
            c if !c.is_ascii() && is_printable(c) => repr.push(c),
            c => {
                let code = u32::from(c);
                repr.push_str(&match code {
                    ..=0xff => format!("\\x{code:02x}"),
                    0x100..=0xffff => format!("\\u{code:04x}"),
                    _ => format!("\\U{code:08x}"),
                });
            }
        }
    }
    repr.push(quote);
    repr
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn whole_numbers_keep_a_decimal_point() {
        assert_eq!([1.0, -0.0, 0.0].map(float_repr), ["1.0", "-0.0", "0.0"]);
    }

    #[test]
    fn decimals_use_the_shortest_round_tripping_digits() {
        assert_eq!(
            [-0.45, 0.1 + 0.2].map(float_repr),
            ["-0.45", "0.30000000000000004"]
        );
    }

    #[test]
    fn small_numbers_switch_to_scientific_notation_below_1e_minus_4() {
        assert_eq!(
            [0.0001, 0.00001, -1.5e-7].map(float_repr),
            ["0.0001", "1e-05", "-1.5e-07"]
        );
    }

    #[test]
    fn large_numbers_switch_to_scientific_notation_from_1e16() {
        assert_eq!(
            [1e15, 1e16, 1.25e300].map(float_repr),
            ["1000000000000000.0", "1e+16", "1.25e+300"]
        );
    }

    #[test]
    fn non_finite_values_are_named_like_python() {
        assert_eq!(
            [f64::NAN, f64::INFINITY, f64::NEG_INFINITY].map(float_repr),
            ["nan", "inf", "-inf"]
        );
    }

    #[test]
    fn lists_are_bracketed_and_comma_separated() {
        assert_eq!(float_list_repr(&[5.0, 52.5]), "[5.0, 52.5]");
    }

    #[test]
    fn ints_accept_spaces_signs_underscores_and_unicode_digits() {
        assert_eq!(
            [
                " -0012 ",
                "+1_000",
                "\u{661}\u{662}",
                "\u{b}7\u{3000}",
                "-0"
            ]
            .map(parse_int),
            [Some("-12"), Some("1000"), Some("12"), Some("7"), Some("0")]
                .map(|v| v.map(str::to_owned))
        );
    }

    #[test]
    fn ints_reject_what_python_rejects() {
        assert_eq!(
            [
                "", " ", "1_", "_1", "1__0", "1.0", "+-1", "1 2", "0x10", "\u{b2}", "\u{1c}7"
            ]
            .map(parse_int),
            [
                None, None, None, None, None, None, None, None, None, None, None
            ]
        );
    }

    #[test]
    fn ints_are_limited_to_4300_digits() {
        assert!(parse_int(&"9".repeat(4300)).is_some());
        assert_eq!(parse_int(&"0".repeat(4301)), None);
    }

    #[test]
    fn floats_accept_python_spellings() {
        assert_eq!(
            [
                " 1_0.5e1_0 ",
                "1.",
                ".5",
                "-1E-3",
                "+inf",
                "-Infinity",
                "\u{661}e1"
            ]
            .map(parse_float),
            [
                Some(105e9),
                Some(1.0),
                Some(0.5),
                Some(-0.001),
                Some(f64::INFINITY),
                Some(f64::NEG_INFINITY),
                Some(10.0)
            ]
        );
        assert!(parse_float("-NaN").is_some_and(f64::is_nan));
    }

    #[test]
    fn floats_reject_what_python_rejects() {
        assert_eq!(
            [
                "", ".", "e5", "1e", "1_.5", "1._5", "_1", "0x1p3", "in f", "nan1", "1e+"
            ]
            .map(parse_float),
            [
                None, None, None, None, None, None, None, None, None, None, None
            ]
        );
    }

    #[test]
    fn strtobool_is_case_insensitive() {
        assert_eq!(
            ["YES", "On", "0", "F", "maybe", ""].map(strtobool),
            [Some(true), Some(true), Some(false), Some(false), None, None]
        );
    }

    #[test]
    fn str_repr_quotes_and_escapes_like_python() {
        assert_eq!(
            [
                "a",
                "it's",
                "it's \"x\"",
                "\\\t\n\r\u{1}\u{7f}",
                "é😀",
                "\u{a0}\u{202e}\u{e000}\u{10ffff}"
            ]
            .map(str_repr),
            [
                "'a'",
                "\"it's\"",
                "'it\\'s \"x\"'",
                "'\\\\\\t\\n\\r\\x01\\x7f'",
                "'é😀'",
                "'\\xa0\\u202e\\ue000\\U0010ffff'",
            ]
        );
    }
}
