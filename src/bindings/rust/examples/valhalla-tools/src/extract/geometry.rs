use super::Error;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rect {
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
}

impl Rect {
    /// Closed test: rectangles that only share an edge or a corner intersect.
    pub(crate) fn intersects(&self, other: &Rect) -> bool {
        self.min_x <= other.max_x
            && other.min_x <= self.max_x
            && self.min_y <= other.max_y
            && other.min_y <= self.max_y
    }
}

/// Parses a `minx,miny,maxx,maxy` bounding box in degrees.
pub fn parse_bbox(text: &str) -> Result<Rect, Error> {
    let invalid = |reason| Error::InvalidBbox {
        text: text.to_owned(),
        reason,
    };
    let coordinates = text
        .split(',')
        .map(|value| value.trim().parse::<f64>())
        .collect::<Result<Vec<_>, _>>()
        .map_err(|_| invalid("expected comma-separated numbers"))?;
    let [min_x, min_y, max_x, max_y] = coordinates[..] else {
        return Err(invalid("expected 4 values: minx,miny,maxx,maxy"));
    };
    if !(-180.0 <= min_x && min_x < max_x && max_x <= 180.0) {
        return Err(invalid("x values must be increasing and within -180..180"));
    }
    if !(-90.0 <= min_y && min_y < max_y && max_y <= 90.0) {
        return Err(invalid("y values must be increasing and within -90..90"));
    }
    Ok(Rect {
        min_x,
        min_y,
        max_x,
        max_y,
    })
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Point {
    pub(crate) x: f64,
    pub(crate) y: f64,
}

/// The exterior ring of a polygon: holes are not kept, so their area counts as covered.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Polygon {
    ring: Vec<Point>,
    /// `None` for an empty ring.
    bounds: Option<Rect>,
}

impl Polygon {
    pub(crate) fn new(ring: Vec<Point>) -> Self {
        let bounds = ring.split_first().map(|(first, rest)| {
            let start = Rect {
                min_x: first.x,
                min_y: first.y,
                max_x: first.x,
                max_y: first.y,
            };
            rest.iter().fold(start, |bounds, point| Rect {
                min_x: bounds.min_x.min(point.x),
                min_y: bounds.min_y.min(point.y),
                max_x: bounds.max_x.max(point.x),
                max_y: bounds.max_y.max(point.y),
            })
        });
        Self { ring, bounds }
    }

    /// Closed test, like `Rect::intersects`: touching the rectangle's boundary counts.
    pub(crate) fn intersects(&self, rect: &Rect) -> bool {
        // Skips the edge loop for the many tiles far from a large region.
        if !self.bounds.is_some_and(|bounds| bounds.intersects(rect)) {
            return false;
        }
        // If no edge reaches the rectangle, it is either fully inside or fully outside.
        self.edges().any(|(a, b)| segment_intersects(a, b, rect))
            || self.contains(Point {
                x: rect.min_x,
                y: rect.min_y,
            })
    }

    /// The ring's edges, including the closing one, which is empty for an explicitly closed ring.
    fn edges(&self) -> impl Iterator<Item = (Point, Point)> + '_ {
        let next = self.ring.iter().cycle().skip(1);
        self.ring.iter().zip(next).map(|(a, b)| (*a, *b))
    }

    /// Even-odd test; the result for points on the boundary is unspecified.
    fn contains(&self, point: Point) -> bool {
        let mut inside = false;
        for (a, b) in self.edges() {
            if (a.y > point.y) != (b.y > point.y) {
                let crossing_x = a.x + (point.y - a.y) * (b.x - a.x) / (b.y - a.y);
                if point.x < crossing_x {
                    inside = !inside;
                }
            }
        }
        inside
    }
}

/// Liang-Barsky clipping of the segment `a`-`b` against the closed rectangle.
fn segment_intersects(a: Point, b: Point, rect: &Rect) -> bool {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let mut enter = 0.0_f64;
    let mut leave = 1.0_f64;
    for (direction, distance) in [
        (-dx, a.x - rect.min_x),
        (dx, rect.max_x - a.x),
        (-dy, a.y - rect.min_y),
        (dy, rect.max_y - a.y),
    ] {
        if direction == 0.0 {
            if distance < 0.0 {
                return false;
            }
        } else if direction < 0.0 {
            enter = enter.max(distance / direction);
        } else {
            leave = leave.min(distance / direction);
        }
    }
    enter <= leave
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

    fn polygon(points: &[(f64, f64)]) -> Polygon {
        Polygon::new(points.iter().map(|&(x, y)| Point { x, y }).collect())
    }

    fn unit_square() -> Rect {
        rect(0.0, 0.0, 1.0, 1.0)
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
    fn a_bbox_with_nan_is_rejected() {
        assert!(parse_bbox("NaN,0,1,1").is_err());
    }

    #[test]
    fn rectangles_sharing_an_edge_intersect() {
        assert!(unit_square().intersects(&rect(1.0, 0.0, 2.0, 1.0)));
    }

    #[test]
    fn rectangles_sharing_a_corner_intersect() {
        assert!(unit_square().intersects(&rect(-1.0, -1.0, 0.0, 0.0)));
    }

    #[test]
    fn a_rectangle_inside_another_intersects_it() {
        assert!(unit_square().intersects(&rect(0.25, 0.25, 0.5, 0.5)));
    }

    #[test]
    fn rectangles_apart_on_x_do_not_intersect() {
        assert!(!unit_square().intersects(&rect(1.5, 0.0, 2.0, 1.0)));
    }

    #[test]
    fn rectangles_apart_on_y_do_not_intersect() {
        assert!(!unit_square().intersects(&rect(0.0, -2.0, 1.0, -0.5)));
    }

    #[test]
    fn a_polygon_with_a_vertex_inside_the_rectangle_intersects_it() {
        let triangle = polygon(&[(0.5, 0.5), (3.0, 0.5), (3.0, 3.0), (0.5, 0.5)]);
        assert!(triangle.intersects(&unit_square()));
    }

    #[test]
    fn a_polygon_inside_the_rectangle_intersects_it() {
        let triangle = polygon(&[(0.2, 0.2), (0.8, 0.2), (0.5, 0.8), (0.2, 0.2)]);
        assert!(triangle.intersects(&unit_square()));
    }

    #[test]
    fn a_rectangle_inside_the_polygon_intersects_it() {
        let large = polygon(&[
            (-5.0, -5.0),
            (5.0, -5.0),
            (5.0, 5.0),
            (-5.0, 5.0),
            (-5.0, -5.0),
        ]);
        assert!(large.intersects(&unit_square()));
    }

    #[test]
    fn a_polygon_crossing_the_rectangle_without_a_vertex_inside_intersects_it() {
        let band = polygon(&[
            (-1.0, 0.4),
            (2.0, 0.4),
            (2.0, 0.6),
            (-1.0, 0.6),
            (-1.0, 0.4),
        ]);
        assert!(band.intersects(&unit_square()));
    }

    #[test]
    fn a_polygon_whose_edge_cuts_a_corner_intersects_the_rectangle() {
        let triangle = polygon(&[(0.5, -0.5), (2.0, -0.5), (2.0, 1.5), (0.5, -0.5)]);
        assert!(triangle.intersects(&unit_square()));
    }

    #[test]
    fn a_disjoint_polygon_does_not_intersect_the_rectangle() {
        let far = polygon(&[(2.0, 2.0), (3.0, 2.0), (3.0, 3.0), (2.0, 2.0)]);
        assert!(!far.intersects(&unit_square()));
    }

    #[test]
    fn a_polygon_around_the_rectangle_corner_without_covering_it_does_not_intersect() {
        // An L wrapped around the square's top-right corner, its inner corner at (1.5, 1.5).
        let l_shape = polygon(&[
            (1.5, -1.0),
            (3.0, -1.0),
            (3.0, 3.0),
            (-1.0, 3.0),
            (-1.0, 1.5),
            (1.5, 1.5),
            (1.5, -1.0),
        ]);
        assert!(!l_shape.intersects(&unit_square()));
    }

    #[test]
    fn a_polygon_touching_the_rectangle_edge_intersects_it() {
        let neighbour = polygon(&[(1.0, 0.0), (2.0, 0.0), (2.0, 1.0), (1.0, 1.0), (1.0, 0.0)]);
        assert!(neighbour.intersects(&unit_square()));
    }

    #[test]
    fn a_polygon_touching_the_rectangle_corner_intersects_it() {
        let triangle = polygon(&[(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 1.0)]);
        assert!(triangle.intersects(&unit_square()));
    }

    #[test]
    fn a_polygon_whose_bounding_box_only_touches_the_rectangle_corner_intersects_it() {
        let wedge = polygon(&[(-2.0, -1.0), (0.0, 0.0), (-1.0, -2.0), (-2.0, -1.0)]);
        assert!(wedge.intersects(&unit_square()));
    }

    #[test]
    fn an_unclosed_ring_is_closed_implicitly() {
        // Only the implicit closing edge from (2, 2) back to (-1, -1) crosses the square.
        let open = polygon(&[(-1.0, -1.0), (-1.0, -3.0), (2.0, -3.0), (2.0, 2.0)]);
        assert!(open.intersects(&unit_square()));
    }

    #[test]
    fn an_empty_ring_intersects_nothing() {
        assert!(!polygon(&[]).intersects(&unit_square()));
    }
}
