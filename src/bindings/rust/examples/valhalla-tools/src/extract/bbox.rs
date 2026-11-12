use super::Error;
use crate::geometry::Rect;

/// Parses a `minx,miny,maxx,maxy` bounding box in degrees. Not elevation's parser: each
/// follows its own script, and this one refuses an empty extent.
pub fn parse_bbox(text: &str) -> Result<Rect, Error> {
    let not_numbers = || Error::BboxNotNumbers(text.to_owned());
    let coordinates = text
        .split(',')
        .map(|value| value.trim().parse::<f64>())
        .collect::<Result<Vec<_>, _>>()
        .map_err(|_| not_numbers())?;
    let [min_x, min_y, max_x, max_y] = coordinates[..] else {
        return Err(not_numbers());
    };
    // Written as "within range" so that NaN fails too.
    let x_valid = -180.0 <= min_x && min_x < max_x && max_x <= 180.0;
    let y_valid = -90.0 <= min_y && min_y < max_y && max_y <= 90.0;
    if !(x_valid && y_valid) {
        return Err(Error::InvalidBbox(coordinates));
    }
    Ok(Rect {
        min_x,
        min_y,
        max_x,
        max_y,
    })
}

#[cfg(test)]
#[expect(clippy::unwrap_used, reason = "tests fail loudly on bad fixtures")]
mod tests {
    use super::*;

    fn rect(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Rect {
        Rect {
            min_x,
            min_y,
            max_x,
            max_y,
        }
    }

    #[test]
    fn a_bbox_with_negative_values_is_parsed() {
        let bbox = parse_bbox("-20,-59.2,-10.2,-53.9").unwrap();
        assert_eq!(bbox, rect(-20.0, -59.2, -10.2, -53.9));
    }

    #[test]
    fn a_bbox_with_spaces_around_values_is_parsed() {
        assert_eq!(parse_bbox("0, 1, 2, 3").unwrap(), rect(0.0, 1.0, 2.0, 3.0));
    }

    #[test]
    fn a_bbox_with_three_values_is_rejected() {
        assert!(parse_bbox("0,1,2").is_err());
    }

    #[test]
    fn a_bbox_with_text_is_rejected() {
        assert!(parse_bbox("0,1,two,3").is_err());
    }

    #[test]
    fn a_bbox_with_min_x_above_max_x_is_rejected() {
        assert!(parse_bbox("3,0,2,1").is_err());
    }

    #[test]
    fn a_bbox_with_equal_min_and_max_y_is_rejected() {
        assert!(parse_bbox("0,1,2,1").is_err());
    }

    #[test]
    fn a_bbox_beyond_the_antimeridian_is_rejected() {
        assert!(parse_bbox("170,0,180.5,1").is_err());
    }

    #[test]
    fn a_bbox_beyond_the_pole_is_rejected() {
        assert!(parse_bbox("0,-90.5,1,1").is_err());
    }

    #[test]
    fn an_out_of_range_bbox_is_reported_with_python_float_reprs() {
        let message = parse_bbox("5,52,4,nan").unwrap_err().to_string();
        assert_eq!(message, "Bbox invalid: [5.0, 52.0, 4.0, nan]");
    }

    #[test]
    fn a_bbox_with_nan_is_rejected() {
        assert!(parse_bbox("NaN,0,1,1").is_err());
    }
}
