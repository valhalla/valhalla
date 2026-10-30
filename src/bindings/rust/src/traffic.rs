use crate::{TileId, TrafficTile, ffi};

/// Real-time traffic data for a single edge, including speeds, congestion levels, and incidents.
/// It is a Rust representation of `valhalla::baldr::TrafficSpeed`.
///
/// It's a copy of the record, so all accessors see the same bits even if the tile is being updated.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LiveTraffic(u64);

impl LiveTraffic {
    /// Live traffic data is unknown for the edge.
    pub const UNKNOWN: Self = Self(0);
    /// Edge is closed due to incident.
    pub const CLOSED: Self = Self(255u64 << 28); // set breakpoint1 to 255, keeping overall_encoded_speed at 0

    /// 7-bit speed value signaling "unknown". Mirrors `UNKNOWN_TRAFFIC_SPEED_RAW` in `traffictile.h`.
    const UNKNOWN_SPEED: u8 = (1 << 7) - 1;
    /// Max raw 6-bit congestion value. Mirrors `MAX_CONGESTION_VAL` in `traffictile.h`.
    const MAX_CONGESTION: u8 = 63;

    /// Constructs a `LiveTraffic` from the raw bits of [`valhalla::baldr::TrafficSpeed`].
    ///
    /// [`valhalla::baldr::TrafficSpeed`]: https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/traffictile.h
    #[inline(always)]
    pub const fn from_bits(value: u64) -> Self {
        Self(value)
    }

    /// Raw bits, laid out as [`valhalla::baldr::TrafficSpeed`].
    ///
    /// [`valhalla::baldr::TrafficSpeed`]: https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/traffictile.h
    #[inline(always)]
    pub const fn to_bits(&self) -> u64 {
        self.0
    }

    /// Creates traffic data from a single, uniform speed for the entire edge. Speeds are stored
    /// with 2 km/h resolution, and 254 or more is stored as unknown.
    #[inline(always)]
    pub const fn from_uniform_speed(speed: u8) -> Self {
        Self::from_segmented_speeds(speed, [speed, 0, 0], [255, 0])
    }

    /// Creates traffic data from multiple speed values for different segments of the edge, stored
    /// like in [`LiveTraffic::from_uniform_speed()`]. Breakpoints are where the first two segments
    /// end, in 1/255 of the edge length. A breakpoint that doesn't advance ends the coverage.
    #[inline(always)]
    pub const fn from_segmented_speeds(
        overall_speed: u8,
        subsegment_speeds: [u8; 3],
        breakpoints: [u8; 2],
    ) -> Self {
        Self(0)
            .with_field(0, 7, (overall_speed >> 1) as u64)
            .with_field(7, 7, (subsegment_speeds[0] >> 1) as u64)
            .with_field(14, 7, (subsegment_speeds[1] >> 1) as u64)
            .with_field(21, 7, (subsegment_speeds[2] >> 1) as u64)
            .with_field(28, 8, breakpoints[0] as u64)
            .with_field(36, 8, breakpoints[1] as u64)
    }

    /// Sets the spare bit to the given value, returning a new `LiveTraffic` instance.
    #[inline(always)]
    pub const fn with_spare(self, spare: bool) -> Self {
        self.with_field(63, 1, spare as u64)
    }

    /// Gets the value of the spare bit.
    #[inline(always)]
    pub const fn spare(&self) -> bool {
        self.field(63, 1) != 0
    }

    /// Overall speed of the edge in km/h, the one Valhalla's costing uses, or `None` without a live
    /// reading. `Some(0)` means closed. Per-segment speeds are in [`LiveTraffic::segments()`].
    ///
    /// ```
    /// use valhalla::LiveTraffic;
    /// assert_eq!(LiveTraffic::from_uniform_speed(72).speed(), Some(72));
    /// assert_eq!(LiveTraffic::UNKNOWN.speed(), None);
    /// ```
    #[inline(always)]
    pub const fn speed(&self) -> Option<u8> {
        let overall_encoded = self.field(0, 7);
        if self.field(28, 8) == 0 || overall_encoded == Self::UNKNOWN_SPEED {
            None
        } else {
            Some(overall_encoded << 1)
        }
    }

    /// Portions of the edge with a live speed, in edge direction. A uniform record has a single
    /// `(0.0, 1.0)` segment, portions without a speed are skipped, and there are no segments when
    /// [`LiveTraffic::speed()`] is `None`.
    #[inline(always)]
    pub fn segments(&self) -> impl Iterator<Item = TrafficSegment> {
        let traffic = *self;
        let (breakpoint1, breakpoint2) = (self.field(28, 8), self.field(36, 8));
        let fences = if self.speed().is_some() {
            [
                (0, breakpoint1),
                (breakpoint1, breakpoint2),
                (breakpoint2, 255),
            ]
        } else {
            [(0, 0); 3]
        };
        // Coverage ends at the first fence that doesn't advance
        fences
            .into_iter()
            .enumerate()
            .take_while(|(_, (start, end))| start < end)
            .filter_map(move |(i, (start, end))| {
                let speed = traffic.field(7 + 7 * i as u32, 7);
                (speed != Self::UNKNOWN_SPEED).then(|| TrafficSegment {
                    range: (start as f32 / 255.0, end as f32 / 255.0),
                    speed: speed << 1,
                    congestion: decode_congestion(traffic.field(44 + 6 * i as u32, 6)),
                })
            })
    }

    /// Whether the edge references incidents in the corresponding incident tile.
    /// Meaningful even without a speed reading.
    #[inline(always)]
    pub const fn has_incidents(&self) -> bool {
        self.field(62, 1) != 0
    }

    /// Sets the per-segment congestion, clamped to `0.0..=1.0`, or `None` for unknown. `Some(1.0)`
    /// marks the segment closed.
    #[inline(always)]
    pub fn with_congestion(self, congestion: [Option<f32>; 3]) -> Self {
        self.with_field(44, 6, encode_congestion(congestion[0]))
            .with_field(50, 6, encode_congestion(congestion[1]))
            .with_field(56, 6, encode_congestion(congestion[2]))
    }

    /// Sets or clears the incidents bit, preserving all other bits. Referencing a real incident
    /// tile is the caller's responsibility.
    #[inline(always)]
    pub const fn with_incidents(self, has_incidents: bool) -> Self {
        self.with_field(62, 1, has_incidents as u64)
    }

    /// `width` bits of the record starting at bit `offset`.
    #[inline(always)]
    const fn field(&self, offset: u32, width: u32) -> u8 {
        ((self.0 >> offset) & ((1 << width) - 1)) as u8
    }

    /// The record with `width` bits starting at bit `offset` replaced by `value`.
    #[inline(always)]
    const fn with_field(self, offset: u32, width: u32, value: u64) -> Self {
        let mask = ((1 << width) - 1) << offset;
        Self((self.0 & !mask) | ((value << offset) & mask))
    }
}

/// A contiguous portion of an edge with its own live-traffic reading. Yielded by
/// [`LiveTraffic::segments()`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrafficSegment {
    /// Portion of the edge this segment covers, as fractions of edge length `(start, end)`, `0.0..=1.0`.
    pub range: (f32, f32),
    /// Speed in km/h, `0` means closed.
    pub speed: u8,
    /// Congestion level `0.0..=1.0`, or `None` if unknown. `1.0` marks the segment closed whatever
    /// its speed.
    pub congestion: Option<f32>,
}

/// Raw `1..=63` maps to `0.0..=1.0`, `0` = unknown.
#[inline(always)]
fn decode_congestion(raw: u8) -> Option<f32> {
    (raw != 0).then(|| (raw as f32 - 1.0) / (LiveTraffic::MAX_CONGESTION as f32 - 1.0))
}

/// The inverse of [`decode_congestion`], clamping to `0.0..=1.0`; non-finite values are unknown.
#[inline(always)]
fn encode_congestion(congestion: Option<f32>) -> u64 {
    match congestion {
        Some(f) if f.is_finite() => {
            (f.clamp(0.0, 1.0) * (LiveTraffic::MAX_CONGESTION as f32 - 1.0)).round() as u64 + 1
        }
        _ => 0,
    }
}

impl TrafficTile {
    /// Id of the graph tile this traffic tile belongs to.
    #[inline(always)]
    pub fn id(&self) -> TileId {
        ffi::id(self).tile()
    }

    /// Seconds since epoch of the last update.
    #[inline(always)]
    pub fn last_update(&self) -> u64 {
        ffi::last_update(self)
    }

    /// Writes the last update timestamp to the memory-mapped file.
    #[inline(always)]
    pub fn write_last_update(&self, unix_timestamp: u64) {
        ffi::write_last_update(self, unix_timestamp)
    }

    /// Custom spare value stored in the header.
    #[inline(always)]
    pub fn spare(&self) -> u64 {
        ffi::spare(self)
    }

    /// Writes a custom value to the spare field in the memory-mapped file.
    #[inline(always)]
    pub fn write_spare(&self, spare: u64) {
        ffi::write_spare(self, spare)
    }

    /// Number of directed edges in this traffic tile.
    #[inline(always)]
    pub fn edge_count(&self) -> u32 {
        self.edge_count
    }

    /// Live traffic information for the given edge index in the tile if available.
    #[inline(always)]
    pub fn edge_traffic(&self, edge_index: u32) -> Option<LiveTraffic> {
        if edge_index < self.edge_count {
            // SAFETY: in bounds, and `traffic_tar` keeps the tile mapped
            let data = unsafe { std::ptr::read_volatile(self.speeds.add(edge_index as usize)) };
            Some(LiveTraffic(data))
        } else {
            None
        }
    }

    /// Writes live traffic information for the given edge index in the tile.
    #[inline(always)]
    pub fn write_edge_traffic(&self, edge_index: u32, traffic: LiveTraffic) {
        if edge_index < self.edge_count {
            // SAFETY: in bounds, and `traffic_tar` keeps the tile mapped
            unsafe { std::ptr::write_volatile(self.speeds.add(edge_index as usize), traffic.0) };
        }
    }

    /// Clears live traffic information in the tile and sets the last update time to 0.
    /// The spare field is left unchanged.
    pub fn clear_traffic(&self) {
        for i in 0..self.edge_count as usize {
            // SAFETY: in bounds, and `traffic_tar` keeps the tile mapped
            unsafe { std::ptr::write_volatile(self.speeds.add(i), 0u64) };
        }
        self.write_last_update(0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pretty_assertions::assert_eq;

    #[test]
    fn test_mock_live_traffic_tile() {
        let mut header: [u64; 16] = [0; 16]; // it should be just big enough. The exact header size is 32 bytes.
        let mut speeds: [u64; 16] = [0; 16];
        let tile = TrafficTile {
            header: header.as_mut_ptr(),
            speeds: speeds.as_mut_ptr(),
            edge_count: 16,
            traffic_tar: cxx::SharedPtr::null(),
        };

        // Initial state with all zeros
        assert_eq!(tile.last_update(), 0);
        assert_eq!(tile.spare(), 0);
        assert_eq!(tile.edge_count(), 16);
        for i in 0..tile.edge_count() {
            assert_eq!(tile.edge_traffic(i), Some(LiveTraffic::UNKNOWN));
        }

        // Out-of-range access
        assert_eq!(tile.edge_traffic(tile.edge_count()), None);
        assert_eq!(tile.edge_traffic(u32::MAX), None);
        let before = speeds;
        tile.write_edge_traffic(tile.edge_count(), LiveTraffic::from_uniform_speed(200));
        tile.write_edge_traffic(u32::MAX, LiveTraffic::from_uniform_speed(200));
        assert_eq!(speeds, before, "out-of-range write must be a no-op");

        tile.write_last_update(1234567890);
        tile.write_spare(42);
        for i in 0..tile.edge_count() {
            tile.write_edge_traffic(i, LiveTraffic::from_uniform_speed(i as u8));
        }

        assert_eq!(tile.last_update(), 1234567890);
        assert_eq!(tile.spare(), 42);
        for i in 0..tile.edge_count() {
            assert_eq!(
                tile.edge_traffic(i),
                Some(LiveTraffic::from_uniform_speed(i as u8))
            );
        }

        for i in 0..tile.edge_count() {
            assert_eq!(
                speeds[i as usize],
                LiveTraffic::from_uniform_speed(i as u8).to_bits()
            );
        }

        // The high bits (congestion, incidents and spare) survive the round-trip too
        let full = LiveTraffic::from_segmented_speeds(72, [60, 80, 100], [127, 200])
            .with_congestion([Some(0.5), None, Some(1.0)])
            .with_incidents(true)
            .with_spare(true);
        tile.write_edge_traffic(3, full);
        assert_eq!(tile.edge_traffic(3), Some(full));
        assert_eq!(speeds[3], full.to_bits());
    }

    #[test]
    fn live_traffic_speed() {
        assert_eq!(LiveTraffic::UNKNOWN.speed(), None);
        assert_eq!(LiveTraffic::CLOSED.speed(), Some(0));
        assert_eq!(LiveTraffic::from_uniform_speed(72).speed(), Some(72));
        // Only the overall speed counts
        assert_eq!(
            LiveTraffic::from_segmented_speeds(72, [60, 80, 100], [127, 200]).speed(),
            Some(72)
        );

        // INVALID_SPEED from traffictile.h
        assert_eq!(LiveTraffic::from_bits(0x0FFF_FFFF).speed(), None);

        // The unknown speed has no reading even with a breakpoint
        let invalid = LiveTraffic::from_bits(
            (LiveTraffic::UNKNOWN_SPEED as u64) // overall_encoded == 127
            | (255u64 << 28), // breakpoint1 != 0
        );
        assert_eq!(invalid.speed(), None);

        // Without a breakpoint there is no reading, so an all-zero record isn't closed
        assert_eq!(LiveTraffic::from_bits(0).speed(), None);
        let no_breakpoint = LiveTraffic::from_bits(36); // overall_encoded == 36, breakpoint1 == 0
        assert_eq!(no_breakpoint.speed(), None);
    }

    #[test]
    fn live_traffic_segments() {
        let segment = |range, speed, congestion| TrafficSegment {
            range,
            speed,
            congestion,
        };

        // No reading, no segments
        assert_eq!(LiveTraffic::UNKNOWN.segments().count(), 0);
        assert_eq!(LiveTraffic::from_bits(0x0FFF_FFFF).segments().count(), 0);

        // CLOSED and uniform records are a single segment over the whole edge
        assert_eq!(
            LiveTraffic::CLOSED.segments().collect::<Vec<_>>(),
            [segment((0.0, 1.0), 0, None)]
        );
        assert_eq!(
            LiveTraffic::from_uniform_speed(72)
                .segments()
                .collect::<Vec<_>>(),
            [segment((0.0, 1.0), 72, None)]
        );

        // A moving, a closed and an unknown portion, the latter is skipped
        let (bp1, bp2) = (100.0 / 255.0, 200.0 / 255.0);
        assert_eq!(
            LiveTraffic::from_segmented_speeds(72, [50, 0, 254], [100, 200])
                .segments()
                .collect::<Vec<_>>(),
            [segment((0.0, bp1), 50, None), segment((bp1, bp2), 0, None),]
        );

        // All segments unknown while the overall speed is known
        let all_unknown = LiveTraffic::from_segmented_speeds(72, [254, 254, 254], [100, 200]);
        assert_eq!(all_unknown.speed(), Some(72));
        assert_eq!(all_unknown.segments().count(), 0);
        // A gap in the middle keeps the segments on both sides of it.
        assert_eq!(
            LiveTraffic::from_segmented_speeds(72, [50, 254, 30], [100, 200])
                .segments()
                .collect::<Vec<_>>(),
            [segment((0.0, bp1), 50, None), segment((bp2, 1.0), 30, None)]
        );

        // breakpoint2 == 255 means two segments, a breakpoint2 that doesn't advance means one
        let two = LiveTraffic::from_segmented_speeds(72, [60, 80, 100], [127, 255]);
        assert_eq!(
            two.segments().collect::<Vec<_>>(),
            [
                segment((0.0, 127.0 / 255.0), 60, None),
                segment((127.0 / 255.0, 1.0), 80, None),
            ]
        );
        for breakpoints in [[127, 0], [100, 80], [100, 100]] {
            let truncated = LiveTraffic::from_segmented_speeds(72, [60, 80, 100], breakpoints);
            assert_eq!(
                truncated.segments().collect::<Vec<_>>(),
                [segment((0.0, breakpoints[0] as f32 / 255.0), 60, None)],
                "breakpoints {breakpoints:?}"
            );
        }
        assert_eq!(
            LiveTraffic::from_segmented_speeds(72, [60, 80, 100], [255, 255])
                .segments()
                .collect::<Vec<_>>(),
            [segment((0.0, 1.0), 60, None)]
        );

        // Raw congestion 0, 1 and 63 are None, Some(0.0) and Some(1.0)
        let bits = LiveTraffic::from_segmented_speeds(72, [60, 80, 100], [100, 200]).to_bits();
        let record = LiveTraffic::from_bits(
            bits | (1u64 << 44) | ((LiveTraffic::MAX_CONGESTION as u64) << 56),
        );
        assert_eq!(
            record.segments().collect::<Vec<_>>(),
            [
                segment((0.0, bp1), 60, Some(0.0)),
                segment((bp1, bp2), 80, None),
                segment((bp2, 1.0), 100, Some(1.0)),
            ]
        );
        // Congestion is quantized to 1/62 steps, so only the endpoints and 0.5 round-trip exactly
        let base = LiveTraffic::from_segmented_speeds(72, [60, 80, 100], [100, 200]);
        let congestion = |t: LiveTraffic| {
            t.segments()
                .map(|segment| segment.congestion)
                .collect::<Vec<_>>()
        };
        let with = base.with_congestion([None, Some(0.0), Some(0.5)]);
        assert_eq!(congestion(with), [None, Some(0.0), Some(0.5)]);
        let clamped = base.with_congestion([Some(-1.0), Some(2.0), None]);
        assert_eq!(congestion(clamped), [Some(0.0), Some(1.0), None]);
        let non_finite =
            base.with_congestion([Some(f32::NAN), Some(f32::INFINITY), Some(f32::NEG_INFINITY)]);
        assert_eq!(congestion(non_finite), [None, None, None]);

        // with_congestion keeps all other bits
        let record = base.with_incidents(true).with_spare(true);
        let with = record.with_congestion([Some(0.25), Some(0.75), None]);
        assert_eq!(with.speed(), record.speed());
        for (w, r) in with.segments().zip(record.segments()) {
            assert_eq!(w.range, r.range);
            assert_eq!(w.speed, r.speed);
        }
        assert!(with.has_incidents() && with.spare());
        assert_eq!(with.with_congestion([None, None, None]), record);
    }

    #[test]
    fn live_traffic_incidents() {
        assert!(!LiveTraffic::UNKNOWN.has_incidents());
        assert!(LiveTraffic::from_bits(1u64 << 62).has_incidents());
        assert!(!LiveTraffic::UNKNOWN.with_spare(true).has_incidents());

        let record = LiveTraffic::from_uniform_speed(72)
            .with_congestion([Some(0.5), None, Some(1.0)])
            .with_spare(true);

        let set = record.with_incidents(true);
        assert!(set.has_incidents());
        assert_eq!(set.speed(), record.speed());
        assert_eq!(
            set.segments().collect::<Vec<_>>(),
            record.segments().collect::<Vec<_>>()
        );
        assert!(set.spare());

        let cleared = set.with_incidents(false);
        assert!(!cleared.has_incidents());
        assert_eq!(cleared, record);
    }
}
