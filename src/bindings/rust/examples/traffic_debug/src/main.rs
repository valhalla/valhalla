//! Debug CLI for Valhalla live traffic tiles.
//!
//! Six operations against a `traffic.tar`:
//!   scan       - fleet sweep: per-tile lines + totals, by-level, freshness histogram
//!   inspect    - per-tile header + summary + edges-with-traffic table
//!   set-speed  - write a uniform speed to one edge
//!   close      - mark one edge as closed
//!   reset      - revert one edge to UNKNOWN (per-edge; tile last_update untouched)
//!   clear      - zero all edges in a tile via `TrafficTile::clear_traffic`
//!
//! Run from the `examples/` workspace:
//!   cargo run -p traffic_debug -- \
//!     --tile-extract path/to/tiles.tar \
//!     --traffic-extract path/to/traffic.tar \
//!     scan

use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, anyhow, bail};
use clap::{Parser, Subcommand};
use valhalla::{ConfigBuilder, GraphId, GraphReader, LiveTraffic, TileId, TrafficSegment};

const SEC_5M: u64 = 5 * 60;
const SEC_30M: u64 = 30 * 60;
const SEC_1H: u64 = 60 * 60;

#[derive(Parser)]
#[command(about = "Inspect and manipulate Valhalla live-traffic.tar files")]
struct Cli {
    /// Path to the Valhalla tile.tar.
    #[arg(long)]
    tile_extract: PathBuf,
    /// Path to the live-traffic.tar.
    #[arg(long)]
    traffic_extract: PathBuf,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Fleet sweep over traffic-tile headers: one line per tile with traffic, then a summary.
    Scan,
    /// Per-tile header + summary + edges-with-traffic table.
    Inspect {
        /// Tile id (`L/T` or `L/T/0` or `0x<hex>`).
        tile_id: String,
    },
    /// Write a uniform speed (kph) to one edge.
    SetSpeed {
        graph_id: String,
        /// Uniform speed in kph, `0..=252` (0 = closed; stored at 2 kph resolution).
        /// Higher values are rejected: 254/255 would encode the "unknown" sentinel.
        #[arg(value_parser = clap::value_parser!(u8).range(..=252))]
        kmh: u8,
        /// Optional overall congestion fraction in `0.0..=1.0`, written to all three subsegments.
        /// Read back via `TrafficSegment::congestion` from `LiveTraffic::segments()`.
        #[arg(long)]
        congestion: Option<f32>,
        /// Mark the edge as referencing incidents (sets the incidents bit).
        #[arg(long)]
        incidents: bool,
    },
    /// Mark one edge as closed (overall=0, breakpoint1=255).
    Close { graph_id: String },
    /// Revert one edge to UNKNOWN. Tile-level last_update is left untouched.
    Reset { graph_id: String },
    /// Zero all edges in a tile via `TrafficTile::clear_traffic`.
    Clear { tile_id: String },
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    run(cli)
}

fn run(cli: Cli) -> Result<()> {
    let reader = open_reader(&cli.tile_extract, &cli.traffic_extract)?;
    match cli.cmd {
        Cmd::Scan => cmd_scan(&reader),
        Cmd::Inspect { tile_id } => cmd_inspect(&reader, &tile_id),
        Cmd::SetSpeed {
            graph_id,
            kmh,
            congestion,
            incidents,
        } => cmd_set_speed(&reader, &graph_id, kmh, congestion, incidents),
        Cmd::Close { graph_id } => cmd_close(&reader, &graph_id),
        Cmd::Reset { graph_id } => cmd_reset(&reader, &graph_id),
        Cmd::Clear { tile_id } => cmd_clear(&reader, &tile_id),
    }
}

fn open_reader(
    tile_extract: &std::path::Path,
    traffic_extract: &std::path::Path,
) -> Result<GraphReader> {
    let config = ConfigBuilder {
        mjolnir: valhalla::config::Mjolnir {
            tile_extract: tile_extract.display().to_string(),
            traffic_extract: traffic_extract.display().to_string(),
            ..Default::default()
        },
        ..Default::default()
    }
    .build();
    GraphReader::new(&config).map_err(|e| anyhow!("failed to open GraphReader: {e}"))
}

const HEARTBEAT_EVERY: usize = 5_000;

fn cmd_scan(reader: &GraphReader) -> Result<()> {
    let now = unix_now()?;

    let mut total_tiles = 0usize;
    let mut total_with_traffic = 0usize;
    let mut by_level: Vec<LevelStats> = Vec::new();
    let mut freshness = FreshnessHistogram::default();

    for tile_id in reader.tiles() {
        total_tiles += 1;
        if total_tiles.is_multiple_of(HEARTBEAT_EVERY) {
            eprintln!("scanned {total_tiles} tiles...");
        }

        let level = tile_id.level() as usize;
        ensure_level(&mut by_level, level);
        by_level[level].total += 1;

        let Some(tile) = reader.traffic_tile(tile_id) else {
            continue;
        };
        let last_update = tile.last_update();
        if last_update == 0 {
            continue; // no live traffic data yet
        }

        total_with_traffic += 1;
        by_level[level].with_traffic += 1;
        freshness.observe(last_update, now);
        println!(
            "tile {tile_id}    last_update={last_update}  ({})    edges={}    spare={}",
            format_age(last_update, now),
            tile.edge_count(),
            tile.spare(),
        );
    }

    if total_with_traffic > 0 {
        println!();
    }
    print_scan_summary(total_with_traffic, total_tiles, &by_level, &freshness);
    Ok(())
}

fn cmd_inspect(reader: &GraphReader, tile_id_arg: &str) -> Result<()> {
    let tile_id = parse_tile_id(tile_id_arg)?;
    let now = unix_now()?;

    let traffic_tile = reader
        .traffic_tile(tile_id)
        .with_context(|| format!("no traffic tile for {tile_id}"))?;
    let graph_tile = reader.graph_tile(tile_id);
    if graph_tile.is_none() {
        eprintln!("warning: no graph tile for {tile_id}; traffic-only inspection");
    }

    let edge_count = traffic_tile.edge_count();
    let last_update = traffic_tile.last_update();
    let header_spare = traffic_tile.spare();

    println!(
        "tile {tile_id}    last_update={last_update} ({})    edges={edge_count}",
        format_age(last_update, now),
    );
    println!("header.spare={header_spare}");

    let edges: Vec<LiveTraffic> = (0..edge_count)
        .filter_map(|i| traffic_tile.edge_traffic(i))
        .collect();
    let summary = summarize_traffic(&edges);
    print_inspect_summary(&summary, edge_count);

    if let Some(gt) = graph_tile.as_ref() {
        print_edges_table(tile_id, &edges, gt);
    }

    Ok(())
}

fn print_edges_table(tile_id: TileId, edges: &[LiveTraffic], graph_tile: &valhalla::GraphTile) {
    let mut rows: Vec<EdgeRow> = Vec::new();
    for (idx, live) in edges.iter().enumerate() {
        if live.speed().is_none() {
            continue; // no reading - nothing to show
        }
        let edge_id = match tile_id.graph_id(idx as u32) {
            Some(g) => g,
            None => continue,
        };
        let de = match graph_tile.directededge(idx as u32) {
            Some(de) => de,
            None => continue,
        };
        let info = graph_tile.edgeinfo(de);
        rows.push(EdgeRow {
            graph_id: edge_id,
            way_id: info.way_id,
            road_class: road_class_str(de.road_class()),
            edge_use: edge_use_str(de.use_type()),
            length_m: de.length(),
            speed: format_speed(*live),
            flags: format_flags(*live),
            congestion: format_congestion(*live),
        });
    }

    if rows.is_empty() {
        return;
    }

    println!();
    println!("edges with traffic data:");

    let w_graph = max_len(rows.iter().map(|r| r.graph_id.to_string()), 8);
    let w_way = max_len(rows.iter().map(|r| r.way_id.to_string()), 6);
    let w_rc = max_len(rows.iter().map(|r| r.road_class.to_string()), 8);
    let w_use = max_len(rows.iter().map(|r| r.edge_use.to_string()), 4);
    let w_len = max_len(rows.iter().map(|r| format!("{} m", r.length_m)), 6);
    let w_speed = max_len(rows.iter().map(|r| r.speed.clone()), 5);
    let w_flags = max_len(rows.iter().map(|r| r.flags.clone()), 5);
    let w_cong = max_len(rows.iter().map(|r| r.congestion.clone()), 10);

    println!(
        "  {:<w_graph$}  {:<w_way$}  {:<w_rc$}  {:<w_use$}  {:>w_len$}  {:<w_speed$}  {:<w_flags$}  {:<w_cong$}",
        "graph_id", "way_id", "road_class", "use", "length", "speed", "flags", "congestion",
    );
    for r in &rows {
        let length_str = format!("{} m", r.length_m);
        println!(
            "  {:<w_graph$}  {:<w_way$}  {:<w_rc$}  {:<w_use$}  {:>w_len$}  {:<w_speed$}  {:<w_flags$}  {:<w_cong$}",
            r.graph_id.to_string(),
            r.way_id,
            r.road_class,
            r.edge_use,
            length_str,
            r.speed,
            r.flags,
            r.congestion,
        );
    }
}

struct EdgeRow {
    graph_id: GraphId,
    way_id: u64,
    road_class: &'static str,
    edge_use: &'static str,
    length_m: u32,
    speed: String,
    flags: String,
    congestion: String,
}

fn max_len(values: impl IntoIterator<Item = String>, min: usize) -> usize {
    values
        .into_iter()
        .map(|v| v.chars().count())
        .max()
        .unwrap_or(min)
        .max(min)
}

fn road_class_str(rc: valhalla::RoadClass) -> &'static str {
    type R = valhalla::RoadClass;
    match rc {
        R::kMotorway => "motorway",
        R::kTrunk => "trunk",
        R::kPrimary => "primary",
        R::kSecondary => "secondary",
        R::kTertiary => "tertiary",
        R::kUnclassified => "unclassified",
        R::kResidential => "residential",
        R::kServiceOther => "service",
        _ => "invalid",
    }
}

fn edge_use_str(u: valhalla::EdgeUse) -> &'static str {
    type U = valhalla::EdgeUse;
    match u {
        U::kRoad => "Road",
        U::kRamp => "Ramp",
        U::kTurnChannel => "TurnChannel",
        U::kTrack => "Track",
        U::kDriveway => "Driveway",
        U::kAlley => "Alley",
        U::kParkingAisle => "ParkingAisle",
        U::kEmergencyAccess => "EmergencyAccess",
        U::kDriveThru => "DriveThru",
        U::kCuldesac => "Culdesac",
        U::kLivingStreet => "LivingStreet",
        U::kServiceRoad => "ServiceRoad",
        U::kCycleway => "Cycleway",
        U::kMountainBike => "MountainBike",
        U::kSidewalk => "Sidewalk",
        U::kFootway => "Footway",
        U::kSteps => "Steps",
        U::kPath => "Path",
        U::kPedestrian => "Pedestrian",
        U::kBridleway => "Bridleway",
        U::kPedestrianCrossing => "PedestrianCrossing",
        U::kElevator => "Elevator",
        U::kEscalator => "Escalator",
        U::kPlatform => "Platform",
        U::kRestArea => "RestArea",
        U::kServiceArea => "ServiceArea",
        U::kOther => "Other",
        U::kFerry => "Ferry",
        U::kRailFerry => "RailFerry",
        U::kConstruction => "Construction",
        U::kRail => "Rail",
        U::kBus => "Bus",
        U::kEgressConnection => "EgressConnection",
        U::kPlatformConnection => "PlatformConnection",
        U::kTransitConnection => "TransitConnection",
        _ => "?",
    }
}

/// Speed cell: blank for no-reading/closed (flags carry "closed"); plain `{kph} kph` for a uniform
/// record; else per-segment, e.g. `0-39%: 50 kph | 39-78%: closed`.
fn format_speed(live: LiveTraffic) -> String {
    if matches!(live.speed(), None | Some(0)) {
        return String::new(); // no reading / closed - flags column carries "closed"
    }
    let cells: Vec<String> = live.segments().map(render_segment).collect();
    cells.join(" | ")
}

/// One segment as `start-end%: state` (range as percents of edge length), e.g. `39-78%: closed`;
/// just the state when the segment covers the whole edge.
fn render_segment(segment: TrafficSegment) -> String {
    let state = match segment.speed {
        0 => "closed".to_string(),
        kph => format!("{kph} kph"),
    };
    match segment.range {
        (0.0, 1.0) => state,
        (start, end) => format!("{:.0}-{:.0}%: {state}", start * 100.0, end * 100.0),
    }
}

/// Flags cell: closed/segmented from the reading, plus the incidents and spare bits.
fn format_flags(live: LiveTraffic) -> String {
    let mut flags = Vec::new();
    if live.speed() == Some(0) {
        flags.push("closed");
    }
    if live.segments().count() > 1 {
        flags.push("segmented");
    }
    if live.has_incidents() {
        flags.push("incidents");
    }
    if live.spare() {
        flags.push("spare");
    }
    flags.join(", ")
}

/// Congestion cell: one `0.00`..`1.00` value per segment joined with `/` (`-` = no data);
/// blank when no segment carries congestion (incl. records without a reading - no segments).
fn format_congestion(live: LiveTraffic) -> String {
    if live.segments().all(|s| s.congestion.is_none()) {
        return String::new();
    }
    let cells: Vec<String> = live
        .segments()
        .map(|s| match s.congestion {
            Some(f) => format!("{f:.2}"),
            None => "-".to_string(),
        })
        .collect();
    cells.join("/")
}

fn parse_tile_id(s: &str) -> Result<TileId> {
    let s = s.trim();
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        let value =
            u64::from_str_radix(hex, 16).with_context(|| format!("invalid hex graph id {s:?}"))?;
        let g = GraphId::new(value);
        if g.id() != 0 {
            bail!("expected tile id, got edge id (raw value resolves to {g})");
        }
        return Ok(g.tile());
    }
    let parts: Vec<&str> = s.split('/').collect();
    match parts.as_slice() {
        [level, tile_index] => Ok(to_graph_id(level, tile_index, "0")?.tile()),
        [level, tile_index, id] => {
            if *id != "0" {
                bail!("expected tile id, got edge id (L/T/I with I != 0); got {s:?}");
            }
            Ok(to_graph_id(level, tile_index, "0")?.tile())
        }
        _ => bail!("expected `L/T` or `L/T/0` (got {s:?})"),
    }
}

fn to_graph_id(level: &str, tile_index: &str, id: &str) -> Result<GraphId> {
    let level: u8 = level
        .parse()
        .with_context(|| format!("invalid level {level:?}"))?;
    let tile_index: u32 = tile_index
        .parse()
        .with_context(|| format!("invalid tile index {tile_index:?}"))?;
    let id: u32 = id
        .parse()
        .with_context(|| format!("invalid edge id {id:?}"))?;
    GraphId::from_parts(level, tile_index, id).ok_or_else(|| {
        anyhow!("invalid graph id parts: level={level} tile_index={tile_index} id={id}")
    })
}

#[derive(Debug, Default, PartialEq, Eq)]
struct TileSummary {
    with_traffic: usize,
    closed: usize,
    segmented: usize,
    with_incidents: usize,
    with_congestion: usize,
    spare_bit_set: usize,
    speeds_kph: Vec<u8>,
}

/// Tally per-edge traffic state via [`LiveTraffic::speed`] and [`LiveTraffic::segments`];
/// all sentinel handling lives inside the binding, none here.
fn summarize_traffic(edges: &[LiveTraffic]) -> TileSummary {
    let mut s = TileSummary::default();
    for e in edges {
        // Orthogonal bits: readable regardless of the reading state.
        if e.has_incidents() {
            s.with_incidents += 1;
        }
        if e.spare() {
            s.spare_bit_set += 1;
        }
        // Reading-scoped detail: congestion lives on segments; "segmented" = more than one.
        if e.segments().any(|segment| segment.congestion.is_some()) {
            s.with_congestion += 1;
        }
        if e.segments().count() > 1 {
            s.segmented += 1;
        }
        match e.speed() {
            None => {} // no reading (incl. the INVALID sentinel)
            // Some(0) = closed - never enters the speed distribution.
            Some(0) => {
                s.with_traffic += 1;
                s.closed += 1;
            }
            Some(kph) => {
                s.with_traffic += 1;
                s.speeds_kph.push(kph);
            }
        }
    }
    s
}

fn print_inspect_summary(s: &TileSummary, total: u32) {
    println!();
    println!(
        "  with traffic data:   {} / {}   ({:.2}%)",
        s.with_traffic,
        total,
        percent(s.with_traffic, total as usize),
    );
    println!("  closed edges:        {}", s.closed);
    println!("  segmented edges:     {}", s.segmented);
    println!("  edges with incidents:      {}", s.with_incidents);
    println!("  edges with congestion:     {}", s.with_congestion);

    let mut sorted = s.speeds_kph.clone();
    sorted.sort_unstable();
    if let Some((p10, p50, p90, max)) = percentiles(&sorted) {
        println!("  speed distribution:  p10={p10}  p50={p50}  p90={p90}  max={max} kph");
    } else {
        println!("  speed distribution:  (no valid speeds)");
    }
    println!("  edges with spare bit set:  {}", s.spare_bit_set);
}

/// Returns (p10, p50, p90, max) for a sorted slice, or None if empty.
fn percentiles(sorted: &[u8]) -> Option<(u8, u8, u8, u8)> {
    if sorted.is_empty() {
        return None;
    }
    let pick = |q: f64| -> u8 {
        // (len - 1) * q rounds to at most len - 1 for q in [0, 1] - always in bounds.
        sorted[((sorted.len() as f64 - 1.0) * q).round() as usize]
    };
    Some((pick(0.10), pick(0.50), pick(0.90), *sorted.last().unwrap()))
}

fn cmd_clear(reader: &GraphReader, tile_id_arg: &str) -> Result<()> {
    let tile_id = parse_tile_id(tile_id_arg)?;
    let traffic_tile = reader
        .traffic_tile(tile_id)
        .with_context(|| format!("no traffic tile for {tile_id}"))?;
    let spare_before = traffic_tile.spare();
    let edge_count = traffic_tile.edge_count();
    traffic_tile.clear_traffic();
    println!("cleared {tile_id} ({edge_count} edges); header.spare preserved = {spare_before}");
    Ok(())
}

fn cmd_set_speed(
    reader: &GraphReader,
    graph_id_arg: &str,
    kmh: u8,
    congestion: Option<f32>,
    incidents: bool,
) -> Result<()> {
    let graph_id = parse_graph_id(graph_id_arg)?;
    let (rounded, lossy) = round_speed_to_2kph(kmh);
    if lossy {
        eprintln!("note: {kmh} kph rounded to {rounded} kph (2 kph resolution)");
    }
    let traffic_tile = reader
        .traffic_tile(graph_id.tile())
        .with_context(|| format!("no traffic tile for {}", graph_id.tile()))?;
    let prev = traffic_tile.edge_traffic(graph_id.id()).with_context(|| {
        format!(
            "edge {graph_id} out of range (tile has {} edges)",
            traffic_tile.edge_count()
        )
    })?;
    let now = unix_now()?;
    let live = build_live_speed(rounded, prev.spare(), congestion, incidents);
    traffic_tile.write_edge_traffic(graph_id.id(), live);
    traffic_tile.write_last_update(now);
    let mut extras = String::new();
    if let Some(c) = congestion {
        extras.push_str(&format!(", congestion={c:.2}"));
    }
    if incidents {
        extras.push_str(", incidents");
    }
    println!(
        "set {graph_id} (0x{:016x}): {rounded} kph{extras}, last_update=now",
        graph_id.value,
    );
    Ok(())
}

/// Uniform-speed record with the optional congestion/incidents modifiers, preserving the previous
/// spare bit; reads back as a single full-coverage segment carrying that congestion.
fn build_live_speed(kmh: u8, spare: bool, congestion: Option<f32>, incidents: bool) -> LiveTraffic {
    LiveTraffic::from_uniform_speed(kmh)
        .with_spare(spare)
        .with_congestion([congestion; 3])
        .with_incidents(incidents)
}

fn cmd_close(reader: &GraphReader, graph_id_arg: &str) -> Result<()> {
    let graph_id = parse_graph_id(graph_id_arg)?;
    let traffic_tile = reader
        .traffic_tile(graph_id.tile())
        .with_context(|| format!("no traffic tile for {}", graph_id.tile()))?;
    let prev = traffic_tile.edge_traffic(graph_id.id()).with_context(|| {
        format!(
            "edge {graph_id} out of range (tile has {} edges)",
            traffic_tile.edge_count()
        )
    })?;
    let now = unix_now()?;
    let live = LiveTraffic::CLOSED.with_spare(prev.spare());
    traffic_tile.write_edge_traffic(graph_id.id(), live);
    traffic_tile.write_last_update(now);
    println!(
        "close {graph_id} (0x{:016x}): CLOSED, last_update=now",
        graph_id.value,
    );
    Ok(())
}

fn cmd_reset(reader: &GraphReader, graph_id_arg: &str) -> Result<()> {
    let graph_id = parse_graph_id(graph_id_arg)?;
    let traffic_tile = reader
        .traffic_tile(graph_id.tile())
        .with_context(|| format!("no traffic tile for {}", graph_id.tile()))?;
    let prev = traffic_tile.edge_traffic(graph_id.id()).with_context(|| {
        format!(
            "edge {graph_id} out of range (tile has {} edges)",
            traffic_tile.edge_count()
        )
    })?;
    let live = LiveTraffic::UNKNOWN.with_spare(prev.spare());
    traffic_tile.write_edge_traffic(graph_id.id(), live);
    // Deliberately do NOT update last_update - per-edge reset preserves tile-level freshness
    // so the rest of the tile's traffic remains "fresh" for the router.
    println!(
        "reset {graph_id} (0x{:016x}): UNKNOWN (tile-level last_update unchanged)",
        graph_id.value,
    );
    Ok(())
}

fn parse_graph_id(s: &str) -> Result<GraphId> {
    let s = s.trim();
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        let value =
            u64::from_str_radix(hex, 16).with_context(|| format!("invalid hex graph id {s:?}"))?;
        return Ok(GraphId::new(value));
    }
    let parts: Vec<&str> = s.split('/').collect();
    match parts.as_slice() {
        [level, tile_index, id] => to_graph_id(level, tile_index, id),
        _ => bail!("expected `L/T/I` (got {s:?})"),
    }
}

/// Returns (rounded_kph, was_lossy). Speeds are stored at 2 kph resolution; input above 252 is
/// rejected by the CLI arg parser (254/255 would encode the "unknown" sentinel).
fn round_speed_to_2kph(kmh: u8) -> (u8, bool) {
    (kmh & !1, kmh & 1 == 1)
}

/// Render `last_update` relative to `now` (e.g. "2m ago", "in 4m future (!)", "empty").
fn format_age(last_update: u64, now: u64) -> String {
    if last_update == 0 {
        return "empty".to_string();
    }
    if last_update > now {
        let diff = last_update - now;
        return format!("in {} future (!)", short_duration(diff));
    }
    format!("{} ago", short_duration(now - last_update))
}

fn short_duration(secs: u64) -> String {
    if secs < 60 {
        format!("{secs}s")
    } else if secs < 3_600 {
        format!("{}m", secs / 60)
    } else if secs < 86_400 {
        format!("{}h", secs / 3_600)
    } else {
        format!("{}d", secs / 86_400)
    }
}

#[derive(Default, Clone)]
struct LevelStats {
    total: usize,
    with_traffic: usize,
}

fn ensure_level(stats: &mut Vec<LevelStats>, level: usize) {
    while stats.len() <= level {
        stats.push(LevelStats::default());
    }
}

#[derive(Debug, Default, PartialEq, Eq)]
struct FreshnessHistogram {
    future: usize,
    /// Counts per age bucket: `< 5 min`, `5-30 min`, `30-60 min`, `> 60 min`.
    by_age: [usize; 4],
}

const FRESHNESS_LABELS: [&str; 4] = ["< 5 min:", "5-30 min:", "30-60 min:", "> 60 min:"];
const FRESHNESS_BOUNDS: [u64; 3] = [SEC_5M, SEC_30M, SEC_1H];

impl FreshnessHistogram {
    /// `last_update == 0` is filtered upstream (means "no traffic").
    fn observe(&mut self, last_update: u64, now: u64) {
        if last_update > now {
            self.future += 1;
            return;
        }
        let age = now - last_update;
        self.by_age[FRESHNESS_BOUNDS.iter().filter(|&&b| age >= b).count()] += 1;
    }
}

fn print_scan_summary(
    with_traffic: usize,
    total: usize,
    by_level: &[LevelStats],
    freshness: &FreshnessHistogram,
) {
    let overall_pct = percent(with_traffic, total);
    println!("{with_traffic} of {total} tiles have live traffic ({overall_pct:.3}%)");

    if !by_level.is_empty() {
        println!();
        println!("by level:");
        let total_w = by_level
            .iter()
            .map(|s| s.total.to_string().len())
            .max()
            .unwrap_or(1);
        let with_w = by_level
            .iter()
            .map(|s| s.with_traffic.to_string().len())
            .max()
            .unwrap_or(1);
        for (level, stats) in by_level.iter().enumerate() {
            if stats.total == 0 {
                continue;
            }
            let pct = percent(stats.with_traffic, stats.total);
            println!(
                "  {level}: {:>with_w$} / {:>total_w$}   ({pct:.2}%)",
                stats.with_traffic, stats.total
            );
        }
    }

    println!();
    println!("freshness:");
    if freshness.future > 0 {
        println!("  {:<10} {:>5}   (!)", "future:", freshness.future);
    }
    for (label, count) in FRESHNESS_LABELS.iter().zip(freshness.by_age) {
        println!("  {label:<10} {count:>5}");
    }
}

fn percent(num: usize, denom: usize) -> f64 {
    if denom == 0 {
        0.0
    } else {
        (num as f64 / denom as f64) * 100.0
    }
}

fn unix_now() -> Result<u64> {
    Ok(SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .context("system clock before unix epoch")?
        .as_secs())
}

#[cfg(test)]
mod tests {
    use super::*;
    use pretty_assertions::assert_eq;

    const NOW: u64 = 1_000_000;

    #[test]
    fn freshness_histogram() {
        let mut h = FreshnessHistogram::default();
        // Future timestamps are counted separately.
        h.observe(NOW + 1, NOW);
        h.observe(NOW + 3600, NOW);
        // Half-open boundaries: age < 5m is fresh; exactly 5m/30m/1h fall into the next bucket.
        h.observe(NOW, NOW);
        h.observe(NOW - (SEC_5M - 1), NOW);
        h.observe(NOW - SEC_5M, NOW);
        h.observe(NOW - SEC_30M, NOW);
        h.observe(NOW - SEC_1H, NOW);
        h.observe(1, NOW); // ancient
        assert_eq!(h.future, 2);
        assert_eq!(h.by_age, [2, 1, 1, 2]);
    }

    #[test]
    fn percent_of_total() {
        assert_eq!(percent(0, 0), 0.0); // zero denominator is not a division
        assert_eq!(percent(5, 0), 0.0);
        assert_eq!(percent(1, 4), 25.0);
        assert_eq!(percent(1, 1000), 0.1);
    }

    #[test]
    fn age_rendering() {
        assert_eq!(format_age(0, NOW), "empty");
        assert_eq!(format_age(NOW - 120, NOW), "2m ago");
        assert_eq!(format_age(NOW + 240, NOW), "in 4m future (!)");
        // short_duration picks the largest fitting unit.
        assert_eq!(short_duration(30), "30s");
        assert_eq!(short_duration(60), "1m");
        assert_eq!(short_duration(3_599), "59m");
        assert_eq!(short_duration(3_600), "1h");
        assert_eq!(short_duration(86_400), "1d");
    }

    #[test]
    fn parse_tile_id_forms() {
        // Accepts `L/T`, `L/T/0` and hex; rejects nonzero edge ids and garbage.
        let tile = parse_tile_id("2/756425").unwrap();
        assert_eq!((tile.level(), tile.tile_index()), (2, 756_425));
        assert_eq!(parse_tile_id("2/756425/0").unwrap(), tile);
        let tile = parse_tile_id("0x0").unwrap();
        assert_eq!((tile.level(), tile.tile_index()), (0, 0));
        let err = parse_tile_id("2/756425/42").unwrap_err().to_string();
        assert!(err.contains("expected tile id"), "got: {err}");
        for bad in ["", "abc", "2/", "2/a/0"] {
            assert!(parse_tile_id(bad).is_err(), "{bad:?} must be rejected");
        }
    }

    #[test]
    fn parse_graph_id_forms() {
        // Requires `L/T/I` (tile-only form is rejected); hex round-trips.
        assert!(parse_graph_id("2/756425").is_err());
        let g = parse_graph_id("2/756425/42").unwrap();
        assert_eq!((g.level(), g.tile_index(), g.id()), (2, 756_425, 42));
        let hex = GraphId::from_parts(2, 756_425, 42).unwrap();
        let parsed = parse_graph_id(&format!("0x{:x}", hex.value)).unwrap();
        assert_eq!(
            (parsed.level(), parsed.tile_index(), parsed.id()),
            (2, 756_425, 42)
        );
    }

    #[test]
    fn percentiles_selection() {
        assert_eq!(percentiles(&[]), None);
        assert_eq!(percentiles(&[42]), Some((42, 42, 42, 42)));
        // n=10: round(9 * q) picks indices 1/5/8.
        let v: Vec<u8> = (1..=10).collect();
        assert_eq!(percentiles(&v), Some((2, 6, 9, 10)));
        // n=100: indices 10/50/89.
        let v: Vec<u8> = (1..=100).map(|x: u32| x as u8).collect();
        assert_eq!(percentiles(&v), Some((11, 51, 90, 100)));
    }

    #[test]
    fn set_speed_input() {
        // Stored at 2 kph resolution - odd inputs round down and report lossy.
        assert_eq!(round_speed_to_2kph(0), (0, false));
        assert_eq!(round_speed_to_2kph(48), (48, false));
        assert_eq!(round_speed_to_2kph(49), (48, true));
        assert_eq!(round_speed_to_2kph(251), (250, true));
        assert_eq!(round_speed_to_2kph(252), (252, false));

        // 253..=255 are rejected up front - 254/255 would encode the 127 "unknown" sentinel and
        // silently read back as `speed() == None`.
        let parse = |kmh: &str| {
            Cli::try_parse_from([
                "traffic_debug",
                "--tile-extract",
                "tiles.tar",
                "--traffic-extract",
                "traffic.tar",
                "set-speed",
                "2/756425/42",
                kmh,
            ])
        };
        assert!(parse("0").is_ok());
        assert!(parse("252").is_ok());
        for kmh in ["253", "254", "255"] {
            let err = parse(kmh).err().expect("must be rejected").to_string();
            assert!(
                err.contains("252"),
                "error should name the limit, got: {err}"
            );
        }
    }

    #[test]
    fn write_records_preserve_spare() {
        // cmd_close/cmd_reset carry the edge's previous spare bit via `.with_spare(prev.spare())`.
        for spare in [true, false] {
            let closed = LiveTraffic::CLOSED.with_spare(spare);
            assert_eq!(closed.speed(), Some(0)); // Some(0) = closed
            assert_eq!(closed.spare(), spare);
        }
        let unknown = LiveTraffic::UNKNOWN.with_spare(true);
        assert_eq!(unknown.speed(), None);
        assert!(unknown.spare());
    }

    #[test]
    fn build_live_speed_record() {
        let live = build_live_speed(72, true, Some(0.25), true);
        assert_eq!(live.speed(), Some(72));
        assert!(live.spare());
        assert!(live.has_incidents());
        // Uniform record -> exactly one full-coverage segment carrying the congestion.
        let segments: Vec<TrafficSegment> = live.segments().collect();
        assert_eq!(segments.len(), 1);
        assert_eq!(segments[0].range, (0.0, 1.0));
        assert_eq!(segments[0].speed, 72);
        // round(0.25 * 62) + 1 = 17 -> (17 - 1) / 62 ~= 0.258
        let got = segments[0].congestion.expect("congestion set");
        assert!((got - 0.25).abs() < 0.02, "got {got}");
        // Speed-only call leaves the orthogonal fields clear.
        let plain = build_live_speed(72, false, None, false);
        assert!(!plain.has_incidents());
        assert!(!plain.spare());
        assert_eq!(
            plain.segments().next().expect("one segment").congestion,
            None
        );
    }

    #[test]
    fn format_speed_cells() {
        // Uniform full coverage renders the plain overall kph; no-reading/closed render blank
        // (the flags column carries "closed").
        assert_eq!(format_speed(LiveTraffic::from_uniform_speed(48)), "48 kph");
        assert_eq!(format_speed(LiveTraffic::UNKNOWN), "");
        assert_eq!(format_speed(LiveTraffic::CLOSED), "");
        // Congestion 1.0 leaves the speed as stored.
        let congested =
            LiveTraffic::from_uniform_speed(72).with_congestion([Some(1.0), None, None]);
        assert_eq!(congested.speed(), Some(72));
        assert_eq!(format_speed(congested), "72 kph");

        // Per-segment form: fences as percents and "closed" per segment; portions without a speed
        // and truncated coverage (garbage bp2 < bp1) are omitted.
        let seg = LiveTraffic::from_segmented_speeds(54, [54, 32, 54], [80, 180]);
        assert_eq!(
            format_speed(seg),
            "0-31%: 54 kph | 31-71%: 32 kph | 71-100%: 54 kph"
        );
        let states = LiveTraffic::from_segmented_speeds(72, [50, 0, 254], [100, 200]);
        assert_eq!(format_speed(states), "0-39%: 50 kph | 39-78%: closed");
        let truncated = LiveTraffic::from_segmented_speeds(72, [50, 60, 70], [100, 80]);
        assert_eq!(format_speed(truncated), "0-39%: 50 kph");
    }

    #[test]
    fn format_flags_cells() {
        assert_eq!(format_flags(LiveTraffic::UNKNOWN), "");
        assert_eq!(format_flags(LiveTraffic::from_uniform_speed(48)), "");
        assert_eq!(format_flags(LiveTraffic::CLOSED), "closed");
        let seg = LiveTraffic::from_segmented_speeds(54, [54, 32, 54], [80, 180]);
        assert_eq!(format_flags(seg), "segmented");
        let spare = LiveTraffic::from_uniform_speed(22).with_spare(true);
        assert_eq!(format_flags(spare), "spare");
        let incidents = LiveTraffic::from_uniform_speed(48).with_incidents(true);
        assert_eq!(format_flags(incidents), "incidents");
        // Flags combine: overall-closed with multiple segments; closed + incidents + spare.
        let seg_closed = LiveTraffic::from_segmented_speeds(0, [0, 32, 54], [80, 180]);
        assert_eq!(seg_closed.speed(), Some(0));
        assert_eq!(format_flags(seg_closed), "closed, segmented");
        let all = LiveTraffic::CLOSED.with_incidents(true).with_spare(true);
        assert_eq!(format_flags(all), "closed, incidents, spare");
    }

    #[test]
    fn format_congestion_cells() {
        // No congestion -> blank.
        assert_eq!(format_congestion(LiveTraffic::from_uniform_speed(48)), "");
        // Uniform record via the demonstrated write path: one segment -> one cell.
        let live = build_live_speed(48, false, Some(0.5), false);
        assert_eq!(format_congestion(live), "0.50");
        // Per-segment cells; `-` marks segments without data.
        let live = LiveTraffic::from_segmented_speeds(72, [50, 60, 70], [100, 200])
            .with_congestion([Some(0.0), None, Some(1.0)]);
        assert_eq!(format_congestion(live), "0.00/-/1.00");
        // No reading -> no segments -> congestion bits unreadable -> blank.
        let live = LiveTraffic::UNKNOWN.with_congestion([Some(0.5), None, None]);
        assert_eq!(format_congestion(live), "");
    }

    #[test]
    fn summarize_traffic_tallies() {
        let edges = vec![
            LiveTraffic::UNKNOWN,                                            // skipped
            LiveTraffic::from_uniform_speed(48),                             // valid uniform
            LiveTraffic::from_uniform_speed(72),                             // valid uniform
            LiveTraffic::CLOSED,                                             // closed
            LiveTraffic::from_segmented_speeds(54, [54, 32, 54], [80, 180]), // segmented
            LiveTraffic::from_uniform_speed(22).with_spare(true),            // spare, also uniform
            LiveTraffic::from_uniform_speed(30).with_incidents(true),        // incidents + speed
            LiveTraffic::from_uniform_speed(40).with_congestion([Some(0.5), None, None]),
        ];
        let s = summarize_traffic(&edges);
        assert_eq!(s.with_traffic, 7); // everything except UNKNOWN
        assert_eq!(s.closed, 1);
        assert_eq!(s.segmented, 1);
        assert_eq!(s.with_incidents, 1);
        assert_eq!(s.with_congestion, 1);
        assert_eq!(s.spare_bit_set, 1);
        // Valid speeds exclude the closed edge.
        let mut speeds = s.speeds_kph.clone();
        speeds.sort();
        assert_eq!(speeds, vec![22, 30, 40, 48, 54, 72]);

        // The 127 INVALID sentinel is no reading: not counted despite a set breakpoint.
        let invalid = LiveTraffic::from_segmented_speeds(254, [10, 20, 30], [80, 180]);
        assert_eq!(invalid.speed(), None);
        let s = summarize_traffic(&[invalid]);
        assert_eq!((s.with_traffic, s.segmented), (0, 0));

        // Orthogonal bits are counted even without a reading; congestion is reading-scoped
        // (no reading -> no segments -> unreadable).
        let edges = vec![
            LiveTraffic::UNKNOWN.with_incidents(true),
            LiveTraffic::UNKNOWN.with_congestion([Some(0.5), None, None]),
            LiveTraffic::UNKNOWN.with_spare(true),
        ];
        let s = summarize_traffic(&edges);
        assert_eq!(s.with_traffic, 0);
        assert_eq!(s.with_incidents, 1);
        assert_eq!(s.with_congestion, 0);
        assert_eq!(s.spare_bit_set, 1);
    }
}
