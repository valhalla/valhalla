use crate::LatLon;

#[inline(always)]
fn zigzag_decode(value: u32) -> i32 {
    (value >> 1) as i32 ^ -((value & 1) as i32)
}

/// Points of an edge shape: zigzag varint deltas at 1e-6 degrees, lat and lon as separate varints.
/// Mirrors `midgard/encoded.h`
#[derive(Clone)]
pub struct Shape<'a> {
    shape: &'a [u8],
    /// Last decoded coordinate, in 1e-6 degrees.
    lat: i32,
    lon: i32,
}

impl<'a> Shape<'a> {
    #[inline(always)]
    pub fn new(shape: &'a [u8]) -> Self {
        Shape {
            shape,
            lat: 0,
            lon: 0,
        }
    }

    #[inline(always)]
    fn varint_decode(&mut self) -> Option<u32> {
        let mut result = 0u32;
        for i in 0..self.shape.len().min(5) {
            let chunk = self.shape[i] as u32;
            result |= (chunk & 0x7f) << (i * 7); // no shift overflow as i < 5
            if chunk & 0x80 == 0 {
                self.shape = &self.shape[i + 1..];
                return Some(result);
            }
        }
        None
    }

    /// Points left, counted without consuming the iterator.
    #[inline(always)]
    pub fn len(&self) -> usize {
        self.shape.iter().filter(|&&byte| byte & 0x80 == 0).count() / 2 // two varints per point
    }

    #[inline(always)]
    pub fn is_empty(&self) -> bool {
        self.shape
            .iter()
            .filter(|&&byte| byte & 0x80 == 0)
            .nth(1)
            .is_none()
    }
}

impl Iterator for Shape<'_> {
    type Item = LatLon;

    #[inline]
    fn next(&mut self) -> Option<LatLon> {
        let lat_change = self.varint_decode()?;
        let lon_change = self.varint_decode()?;
        self.lat += zigzag_decode(lat_change);
        self.lon += zigzag_decode(lon_change);
        Some(LatLon(self.lat as f64 * 1e-6, self.lon as f64 * 1e-6))
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        // With 6-digit precision the max value encoded takes log2(360.0 * 1e6) - 28.4 bits,
        // which gives at most 8 bytes per point - even cross meridian
        let len = self.shape.len();
        (len / 8, Some(len / 2))
    }

    #[inline(always)]
    fn count(self) -> usize {
        self.len()
    }
}

impl std::iter::FusedIterator for Shape<'_> {}

#[cfg(test)]
mod tests {
    use super::*;
    use pretty_assertions::assert_eq;

    fn encode_varint(value: u32, out: &mut Vec<u8>) {
        let mut value = value;
        while value >= 0x80 {
            out.push((value as u8) | 0x80);
            value >>= 7;
        }
        out.push(value as u8);
    }

    fn zigzag_encode(value: i32) -> u32 {
        ((value << 1) ^ (value >> 31)) as u32
    }

    fn encode_shape(points: &[(f64, f64)]) -> Vec<u8> {
        let (mut lat, mut lon) = (0i32, 0i32);
        let mut out = Vec::new();
        for &(plat, plon) in points {
            let (next_lat, next_lon) = ((plat * 1e6) as i32, (plon * 1e6) as i32);
            encode_varint(zigzag_encode(next_lat - lat), &mut out);
            encode_varint(zigzag_encode(next_lon - lon), &mut out);
            (lat, lon) = (next_lat, next_lon);
        }
        out
    }

    #[test]
    fn shape_round_trip() {
        let points = [
            (42.493898, 1.500757),
            (42.493855, 1.500668),
            (42.493785, 1.500565),
        ];
        let encoded = encode_shape(&points);
        let shape = Shape::new(&encoded);

        assert_eq!(shape.len(), points.len());
        assert_eq!(shape.clone().count(), points.len());
        assert!(!shape.is_empty());
        let (lo, hi) = shape.size_hint();
        assert!(lo <= points.len() && points.len() <= hi.unwrap());

        let decoded: Vec<_> = shape.collect();
        assert_eq!(decoded.len(), points.len());
        for (decoded, &(lat, lon)) in decoded.iter().zip(&points) {
            assert!((decoded.0 - lat).abs() < 1e-9, "{decoded:?} != {lat}");
            assert!((decoded.1 - lon).abs() < 1e-9, "{decoded:?} != {lon}");
        }

        // extreme case for size_hint - shape that crosses antimeridian
        let encoded = encode_shape(&[(179.9, 89.9), (-179.9, -89.9), (179.9, 89.9)]);
        assert_eq!(encoded.len(), 29);
        let shape = Shape::new(&encoded);
        assert_eq!(shape.size_hint(), (3, Some(14)));

        // another extreme case - geometry near equator
        let encoded = encode_shape(&[(0.0, 0.0), (0.000001, 0.0), (0.000001, 0.000001)]);
        assert_eq!(encoded.len(), 6);
        let shape = Shape::new(&encoded);
        assert_eq!(shape.size_hint(), (0, Some(3)));
    }

    #[test]
    fn empty_shape() {
        let mut empty = Shape::new(&[]);
        assert!(empty.is_empty());
        assert_eq!(empty.len(), 0);
        assert_eq!(empty.size_hint(), (0, Some(0)));
        assert_eq!(empty.next(), None);
    }
}
