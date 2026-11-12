//! Python's text representations, for messages that must read as the scripts' do.

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
}
