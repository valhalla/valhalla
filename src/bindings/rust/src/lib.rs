use std::{fmt, ptr::NonNull};

use bitflags::bitflags;
use cxx::ExternType;

#[cfg(feature = "proto")]
mod actor;
pub mod config;
mod encoded;
#[cfg(feature = "proto")]
pub mod proto;
mod traffic;

#[cfg(feature = "proto")]
pub use actor::{Actor, Response};
pub use config::Config;
pub use config::ConfigBuilder;
pub use encoded::Shape;
pub use ffi::AdminInfo;
pub use ffi::EdgeInfo;
pub use ffi::EdgeUse;
pub use ffi::NodeType;
pub use ffi::RoadClass;
pub use ffi::TileId;
pub use ffi::TimeZoneInfo;
pub use ffi::TrafficTile;
pub use ffi::decode_weekly_speeds;
pub use ffi::encode_weekly_speeds;
pub use traffic::{LiveTraffic, TrafficSegment};

/// Named hierarchy levels of the default Valhalla graph, as returned by [`GraphId::level`].
pub mod level {
    pub const HIGHWAY: u8 = 0;
    pub const ARTERIAL: u8 = 1;
    pub const LOCAL: u8 = 2;
}

#[cxx::bridge]
mod ffi {
    /// Edge use type. Indicates specialized uses.
    #[namespace = "valhalla::baldr"]
    #[cxx_name = "Use"]
    #[repr(u8)]
    #[derive(Debug)]
    enum EdgeUse {
        // Road specific uses
        kRoad = 0,
        kRamp = 1,            // Link - exits/entrance ramps.
        kTurnChannel = 2,     // Link - turn lane.
        kTrack = 3,           // Agricultural use, forest tracks
        kDriveway = 4,        // Driveway/private service
        kAlley = 5,           // Service road - limited route use
        kParkingAisle = 6,    // Access roads in parking areas
        kEmergencyAccess = 7, // Emergency vehicles only
        kDriveThru = 8,       // Commercial drive-thru (banks/fast-food)
        kCuldesac = 9,        // Cul-de-sac - dead-end road with possible circular end
        kLivingStreet = 10,   // Streets with preference towards bicyclists and pedestrians
        kServiceRoad = 11,    // Generic service road (not driveway, alley, parking aisle, etc.)

        // Bicycle specific uses
        kCycleway = 20,     // Dedicated bicycle path
        kMountainBike = 21, // Mountain bike trail

        kSidewalk = 24,

        // Pedestrian specific uses
        kFootway = 25,
        kSteps = 26, // Stairs
        kPath = 27,
        kPedestrian = 28,
        kBridleway = 29,
        kPedestrianCrossing = 32, // cross walks
        kElevator = 33,
        kEscalator = 34,
        kPlatform = 35,

        // Rest/Service Areas
        kRestArea = 30,
        kServiceArea = 31,

        // Other... currently, either BSS Connection or unspecified service road
        kOther = 40,

        // Ferry and rail ferry
        kFerry = 41,
        kRailFerry = 42,

        kConstruction = 43, // Road under construction

        // Transit specific uses. Must be last in the list
        kRail = 50,               // Rail line
        kBus = 51,                // Bus line
        kEgressConnection = 52,   // Connection between transit station and transit egress
        kPlatformConnection = 53, // Connection between transit station and transit platform
        kTransitConnection = 54,  // Connection between road network and transit egress
    }

    /// [Road class] or importance of an edge.
    ///
    /// [Road class]: https://wiki.openstreetmap.org/wiki/Key:highway#Roads
    #[namespace = "valhalla::baldr"]
    #[repr(u8)]
    #[derive(Debug, PartialOrd, Ord)]
    enum RoadClass {
        kMotorway = 0,
        kTrunk = 1,
        kPrimary = 2,
        kSecondary = 3,
        kTertiary = 4,
        kUnclassified = 5,
        kResidential = 6,
        kServiceOther = 7,
        /// [`DirectedEdge`] has only 3 bits for road class.
        kInvalid = 8,
    }

    /// Type of the node, mostly describing barriers and transit infrastructure.
    #[namespace = "valhalla::baldr"]
    #[repr(u8)]
    #[derive(Debug)]
    enum NodeType {
        /// Regular intersection of 2 roads. The default for any node without a more specific tag.
        kStreetIntersection = 0,
        /// `barrier` = `gate`, `yes`, `lift_gate`, `swing_gate` or `sliding_beam`, or `bollard=rising`.
        kGate = 1,
        /// Fixed obstruction: `barrier` = `bollard`, `block`, `chain`, `bar`, `kissing_gate`,
        /// `cycle_barrier` or `motorcycle_barrier`, or `bollard=removable`.
        kBollard = 2,
        /// `barrier=toll_booth`.
        kTollBooth = 3,
        /// Transit egress, from GTFS feeds rather than OSM.
        kTransitEgress = 4,
        /// Transit station, from GTFS feeds rather than OSM.
        kTransitStation = 5,
        /// Multi-use transit platform (rail and bus), from GTFS feeds rather than OSM.
        kMultiUseTransitPlatform = 6,
        /// `amenity=bicycle_rental`, or `shop=bicycle` with `service:bicycle:rental=yes`.
        kBikeShare = 7,
        /// `amenity=parking`.
        kParking = 8,
        /// `highway=motorway_junction`.
        kMotorWayJunction = 9,
        /// `barrier=border_control`.
        kBorderControl = 10,
        /// `highway=toll_gantry`. Unlike [`NodeType::kTollBooth`], it carries no transition cost
        /// in Valhalla's costing.
        kTollGantry = 11,
        /// `barrier=sump_buster`.
        kSumpBuster = 12,
        /// `entrance=yes` with `indoor=yes`.
        kBuildingEntrance = 13,
        /// `highway=elevator`.
        kElevator = 14,
    }

    /// Dynamic (cold) information about the edge, such as OSM Way ID, speed limit, shape, elevation, etc.
    /// N.B.: Check [`DirectedEdge::forward()`] before reading the shape.
    #[derive(Clone, Copy)]
    struct EdgeInfo<'a> {
        /// OSM Way ID of the edge.
        way_id: u64,
        /// Speed limit in km/h. 0 if not available and 255 if not limited (e.g. autobahn).
        speed_limit: u8,
        /// Shape in Valhalla's 7-bit varint delta encoding.
        encoded_shape: &'a [u8],
    }

    /// Helper struct to pass coordinates in (lat, lon) format between C++ and Rust.
    struct LatLon {
        lat: f64,
        lon: f64,
    }

    /// Identifies a tile: its hierarchy level and its index within that level.
    #[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    struct TileId {
        /// `level | tile_index << 3`, as C++ `GraphId::tile_value()`.
        value: u32,
    }

    /// Information about the administrative area, such as country or state.
    #[derive(Clone)]
    struct AdminInfo {
        /// Text name of the country or "None" if not available.
        country_text: String,
        /// Text name of the state or "None" if not available. May be empty if country has no states.
        state_text: String,
        /// ISO 3166-1 alpha-2 country code.
        country_iso: String,
        /// ISO 3166-2 subdivision code (state/province part only), e.g. 'CA' for 'US-CA'.
        state_iso: String,
    }

    /// Information about the timezone, such as name and offset from UTC.
    #[derive(Clone)]
    struct TimeZoneInfo {
        /// Timezone name in the tz database.
        name: String,
        /// Offset in seconds from UTC for the timezone.
        offset_seconds: i32,
    }

    /// An interface for reading and writing live traffic information for the corresponding graph tile.
    ///
    /// Can be obtained via [`crate::GraphReader::traffic_tile()`].
    /// `TrafficTile` can outlive the [`GraphReader`] that created it.
    struct TrafficTile {
        /// Pointer to [`valhalla::baldr::TrafficTileHeader`] of the tile.
        ///
        /// [`valhalla::baldr::TrafficTileHeader`]: https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/traffictile.h
        header: *mut u64,
        /// Pointer to the start of the array of [`valhalla::baldr::TrafficSpeed`] records for the tile.
        ///
        /// [`valhalla::baldr::TrafficSpeed`]: https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/traffictile.h
        speeds: *mut u64,
        /// Number of directed edges in the tile and thus number of `TrafficSpeed` records.
        edge_count: u32,
        /// Shared ownership of the underlying memory-mapped file with all traffic tiles.
        traffic_tar: SharedPtr<tar>,
    }

    unsafe extern "C++" {
        include!("valhalla/src/bindings/rust/src/libvalhalla.h");

        #[namespace = "valhalla::baldr"]
        type GraphId = crate::GraphId;
        /// Constructs a new `GraphId` from the given hierarchy level, tile index, and unique ID within the tile.
        fn from_parts(level: u8, tile_index: u32, id: u32) -> Result<GraphId>;

        #[namespace = "boost::property_tree"]
        type ptree = crate::config::ffi::ptree;

        type TileSet;
        fn new_tileset(config: &ptree) -> Result<SharedPtr<TileSet>>;
        fn tiles(self: &TileSet) -> Vec<TileId>;
        fn tiles_in_bbox(
            self: &TileSet,
            min_lat: f32,
            min_lon: f32,
            max_lat: f32,
            max_lon: f32,
            level: u8,
        ) -> Vec<TileId>;
        // As cxx doesn't support `boost::intrusive_ptr<T>`, `GraphTile` lifetime should be manually
        // managed by calling [`ffi::add_ref()`] and [`ffi::release()`].
        fn get_graph_tile(self: &TileSet, id: TileId) -> *const GraphTile;
        fn get_traffic_tile(self: &TileSet, id: TileId) -> Result<TrafficTile>;
        fn dataset_id(self: &TileSet) -> u64;

        #[namespace = "valhalla::baldr"]
        type GraphTile;
        // Increases the reference count.
        unsafe fn add_ref(tile: *const GraphTile);
        // Decreases the reference count. `GraphTile` is deleted when it reaches zero.
        unsafe fn release(tile: *const GraphTile);
        fn id(self: &GraphTile) -> GraphId;
        // Returned slice works only because of the `data: [u64; 6]` definition in [`ffi::DirectedEdge`].
        fn directededges(tile: &GraphTile) -> &[DirectedEdge];
        fn directededge(self: &GraphTile, index: usize) -> Result<*const DirectedEdge>;
        fn edgeinfo<'a>(tile: &'a GraphTile, de: &DirectedEdge) -> EdgeInfo<'a>;
        // Returned slice works only because of the `data: [u64; 4]` definition in [`ffi::NodeInfo`].
        fn nodes(tile: &GraphTile) -> &[NodeInfo];
        fn node(self: &GraphTile, index: usize) -> Result<*const NodeInfo>;
        fn node_edges<'a>(tile: &'a GraphTile, node: &NodeInfo) -> &'a [DirectedEdge];
        fn node_transitions<'a>(tile: &'a GraphTile, node: &NodeInfo) -> &'a [NodeTransition];
        fn node_latlon(tile: &GraphTile, node: &NodeInfo) -> LatLon;
        fn admininfo(tile: &GraphTile, index: u32) -> Result<AdminInfo>;
        unsafe fn GetSpeed(
            self: &GraphTile,
            de: *const DirectedEdge,
            flow_mask: u8,
            seconds: u64,
            is_truck: bool,
            flow_sources: *mut u8,
            seconds_from_now: u64,
        ) -> u32;
        // Helper method that returns the raw `TrafficSpeed` bits for the edge's live traffic record.
        fn live_traffic(tile: &GraphTile, de: &DirectedEdge) -> u64;

        #[namespace = "valhalla::midgard"]
        type tar;

        /// Base `GraphId` of the graph tile this traffic tile belongs to.
        fn id(tile: &TrafficTile) -> GraphId;
        /// Seconds since epoch of the last update.
        fn last_update(tile: &TrafficTile) -> u64;
        /// Writes the last update timestamp to the memory-mapped file.
        fn write_last_update(tile: &TrafficTile, unix_timestamp: u64);
        /// Custom spare value stored in the header.
        fn spare(tile: &TrafficTile) -> u64;
        /// Writes a custom value to the spare field in the memory-mapped file.
        fn write_spare(tile: &TrafficTile, spare: u64);

        #[namespace = "valhalla::baldr"]
        #[cxx_name = "Use"]
        type EdgeUse;

        #[namespace = "valhalla::baldr"]
        type RoadClass;

        #[namespace = "valhalla::baldr"]
        type NodeType;

        #[namespace = "valhalla::baldr"]
        type DirectedEdge = crate::DirectedEdge;
        /// End node of the directed edge. [`DirectedEdge::leaves_tile()`] returns true if end node is in a different tile.
        ///
        /// # Examples
        ///
        /// ```
        /// # fn example(reader: &valhalla::GraphReader, tile: &valhalla::GraphTile, edge: &valhalla::DirectedEdge) -> Option<()> {
        /// let end_node_id = edge.endnode();
        /// // Alternatively, check that `end_node_id.tile()` is different from `tile.id()`.
        /// let end_tile = if edge.leaves_tile() {
        ///     reader.graph_tile(end_node_id)?
        /// } else {
        ///     tile.clone()  // `clone()`
        /// };
        /// let end_node = end_tile.node(end_node_id.id())?;
        /// # Some(())
        /// # }
        /// ```
        fn endnode(self: &DirectedEdge) -> GraphId;
        /// The index of the opposing directed edge at the end node of this directed edge.
        ///
        /// # Examples
        ///
        /// ```
        /// # fn example(reader: &valhalla::GraphReader, tile: &valhalla::GraphTile, edge: &valhalla::DirectedEdge) -> Option<()> {
        /// let end_node_id = edge.endnode();
        /// // Alternatively, check that `end_node_id.tile()` is different from `tile.id()`.
        /// let end_tile = if edge.leaves_tile() {
        ///     reader.graph_tile(end_node_id)?
        /// } else {
        ///     tile.clone()
        /// };
        /// let end_node = end_tile.node(end_node_id.id())?;
        /// let opp_edge = &end_tile.node_edges(end_node)[edge.opp_index() as usize];
        /// # Some(())
        /// # }
        /// ```
        fn opp_index(self: &DirectedEdge) -> u32;
        /// Whether this edge is stored forward in [`crate::EdgeInfo`], which both edges of a pair
        /// share. The reverse edge walks the stored shape and elevation backwards.
        fn forward(self: &DirectedEdge) -> bool;
        /// Specialized use type of the edge.
        #[cxx_name = "use"]
        fn use_type(self: &DirectedEdge) -> EdgeUse;
        /// Road class or importance of the edge.
        #[cxx_name = "classification"]
        fn road_class(self: &DirectedEdge) -> RoadClass;
        /// Length of the edge in meters.
        fn length(self: &DirectedEdge) -> u32;
        /// Whether this edge is part of a toll road.
        fn toll(self: &DirectedEdge) -> bool;
        /// Whether this edge is private or destination-only access.
        fn destonly(self: &DirectedEdge) -> bool;
        /// Whether this edge is part of a tunnel.
        fn tunnel(self: &DirectedEdge) -> bool;
        /// Whether this edge is part of a bridge.
        fn bridge(self: &DirectedEdge) -> bool;
        /// Whether this edge is part of a roundabout.
        fn roundabout(self: &DirectedEdge) -> bool;
        /// Whether this edge crosses a country border.
        #[cxx_name = "ctry_crossing"]
        fn crosses_country_border(self: &DirectedEdge) -> bool;
        /// Access modes in the forward direction. Bit mask using [`crate::Access`] constants.
        #[cxx_name = "forwardaccess"]
        fn forwardaccess_u32(self: &DirectedEdge) -> u32;
        /// Access modes in the reverse direction. Bit mask using [`crate::Access`] constants.
        #[cxx_name = "reverseaccess"]
        fn reverseaccess_u32(self: &DirectedEdge) -> u32;
        /// Default speed in km/h for this edge.
        fn speed(self: &DirectedEdge) -> u32;
        /// Truck speed in km/h for this edge.
        fn truck_speed(self: &DirectedEdge) -> u32;
        /// Free flow speed (typical speed during night, from 7pm to 7am) in km/h for this edge.
        fn free_flow_speed(self: &DirectedEdge) -> u32;
        /// Constrained flow speed (typical speed during day, from 7am to 7pm) in km/h for this edge.
        fn constrained_flow_speed(self: &DirectedEdge) -> u32;
        /// Whether this edge is a shortcut edge.
        fn is_shortcut(self: &DirectedEdge) -> bool;
        /// Whether this directed edge ends in a different tile.
        fn leaves_tile(self: &DirectedEdge) -> bool;

        #[namespace = "valhalla::baldr"]
        type NodeInfo = crate::NodeInfo;
        /// Get the index of the first outbound edge from this node. Since all outbound edges are
        /// in the same tile/level as the node we only need an index within the tile.
        fn edge_index(self: &NodeInfo) -> u32;
        /// Get the number of outbound directed edges from this node on the current hierarchy level.
        fn edge_count(self: &NodeInfo) -> u32;
        /// Type of the node, e.g. gate, bollard or toll booth.
        #[cxx_name = "type"]
        fn node_type(self: &NodeInfo) -> NodeType;
        /// Elevation of the node in meters. Returns `-500.0` if elevation data is not available.
        fn elevation(self: &NodeInfo) -> f32;
        /// Access modes allowed to pass through the node. Bit mask using [`crate::Access`] constants.
        #[cxx_name = "access"]
        fn access_u16(self: &NodeInfo) -> u16;
        /// Whether the node is tagged by `access=private`, e.g. a service area gate.
        /// [`NodeInfo::access()`] stays fully open for such nodes, so this is the only way to spot them.
        fn private_access(self: &NodeInfo) -> bool;
        /// Index of the administrative area (country) the node is in. Corresponding [`crate::AdminInfo`] can be
        /// retrieved using [`crate::GraphTile::admin_info()`].
        fn admin_index(self: &NodeInfo) -> u32;
        /// Time zone index of the node. Corresponding [`crate::TimeZoneInfo`] can be retrieved
        /// using [`crate::TimeZoneInfo::from_id()`].
        fn timezone(self: &NodeInfo) -> u32;
        /// Relative road density in the area surrounding the node [0,15]. Higher values indicate more roads nearby.
        /// 15: Avenue des Champs-Elysees in Paris.
        /// 10: Lombard Street in San Francisco.
        /// 9: Unter den Linden in Berlin.
        /// 3: Golden Gate Bridge in San Francisco at the southern end.
        /// 0: Any rural area.
        fn density(self: &NodeInfo) -> u32;
        /// Get the index of the first transition from this node.
        fn transition_index(self: &NodeInfo) -> u32;
        /// Get the number of transitions from this node.
        fn transition_count(self: &NodeInfo) -> u32;

        /// Retrieves the timezone information by its index. `unix_timestamp` is required to handle DST/SDT.
        fn from_id(id: u32, unix_timestamp: u64) -> Result<TimeZoneInfo>;

        #[namespace = "valhalla::baldr"]
        type NodeTransition = crate::NodeTransition;
        /// Graph id of the corresponding node on another hierarchy level.
        fn endnode(self: &NodeTransition) -> GraphId;
        /// Is the transition up to a higher level.
        #[cxx_name = "up"]
        fn upward(self: &NodeTransition) -> bool;

        /// Encodes weekly speed data into a DCT-II compressed base64 string for Valhalla [historical traffic].
        ///
        /// Takes 2016 speed values (one per 5-minute interval covering a full week starting from
        /// Sunday 00:00) and returns a base64-encoded DCT-II compressed representation suitable for
        /// the `valhalla_add_predicted_traffic` tool's CSV input.
        /// N.B.: The encoding is lossy (2016 -> 200 coefficients). Use [`decode_weekly_speeds`] to
        /// evaluate compression quality if needed.
        ///
        /// # Examples
        /// ```
        /// // Generate sample weekly speed profile (constant 50 km/h)
        /// let speeds = vec![50.0; 2016];
        /// let encoded = valhalla::encode_weekly_speeds(&speeds).expect("Failed to encode");
        /// // Use in CSV: "1/47701/130,50,40,{encoded}"
        /// ```
        ///
        /// [historical traffic]: https://valhalla.github.io/valhalla/mjolnir/historical_traffic/#historical-traffic
        fn encode_weekly_speeds(speeds: &[f32]) -> Result<String>;

        /// Decodes a string from [`encode_weekly_speeds`] back to 2016 speed values, one per 5-minute
        /// interval of a week starting from Sunday 00:00.
        fn decode_weekly_speeds(encoded: &str) -> Result<Vec<f32>>;
    }
}

// Safety: All operations do not mutate [`TileSet`] inner state and underlying resources are
// managed by C++ `std::shared_ptr`.
unsafe impl Send for ffi::TileSet {}
unsafe impl Sync for ffi::TileSet {}

/// Identifier of a node or an edge within the tiled, hierarchical graph.
/// Includes the tile Id, hierarchy level, and a unique identifier within the tile/level.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
#[repr(C)]
pub struct GraphId {
    pub value: u64,
}

unsafe impl ExternType for GraphId {
    type Id = cxx::type_id!("valhalla::baldr::GraphId");
    type Kind = cxx::kind::Trivial;
}

impl Default for GraphId {
    #[inline(always)]
    fn default() -> Self {
        Self {
            // `valhalla::baldr::kInvalidGraphId`
            value: 0x3fffffffffff,
        }
    }
}

impl fmt::Debug for GraphId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GraphId")
            .field("level", &self.level())
            .field("tile_index", &self.tile_index())
            .field("id", &self.id())
            .finish()
    }
}

impl fmt::Display for GraphId {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}/{}/{}", self.level(), self.tile_index(), self.id())
    }
}

impl GraphId {
    #[inline(always)]
    pub fn new(value: u64) -> Self {
        Self { value }
    }

    /// Constructs a new `GraphId` from the given hierarchy level, tile index, and unique ID within the tile.
    /// Returns `None` if any of the parts is out of range.
    #[inline(always)]
    pub fn from_parts(level: u8, tile_index: u32, id: u32) -> Option<Self> {
        ffi::from_parts(level, tile_index, id).ok()
    }

    /// Hierarchy level of the tile this identifier belongs to.
    #[inline(always)]
    pub fn level(&self) -> u8 {
        (self.value & 0x7) as u8
    }

    /// Index of the tile within its hierarchy level, C++ `GraphId::tileid()`.
    #[inline(always)]
    pub fn tile_index(&self) -> u32 {
        ((self.value & 0x1fffff8) >> 3) as u32
    }

    /// The tile this identifier belongs to.
    #[inline(always)]
    pub fn tile(&self) -> TileId {
        TileId {
            value: (self.value & 0x1ffffff) as u32,
        }
    }

    /// Identifier within the tile, unique within the tile and level.
    #[inline(always)]
    pub fn id(&self) -> u32 {
        ((self.value & 0x3ffffe000000) >> 25) as u32
    }
}

impl TileId {
    /// Hierarchy level of the tile.
    #[inline(always)]
    pub fn level(&self) -> u8 {
        (self.value & 0x7) as u8
    }

    /// Index of the tile within its hierarchy level.
    #[inline(always)]
    pub fn tile_index(&self) -> u32 {
        self.value >> 3
    }

    /// Identifier of a node or an edge within this tile. Returns `None` if `id` is out of range.
    #[inline(always)]
    pub fn graph_id(&self, id: u32) -> Option<GraphId> {
        GraphId::from_parts(self.level(), self.tile_index(), id)
    }
}

/// The invalid tile, the one [`GraphId::default()`] belongs to.
impl Default for TileId {
    #[inline(always)]
    fn default() -> Self {
        GraphId::default().tile()
    }
}

impl From<GraphId> for TileId {
    #[inline(always)]
    fn from(id: GraphId) -> Self {
        id.tile()
    }
}

impl fmt::Debug for TileId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("TileId")
            .field("level", &self.level())
            .field("tile_index", &self.tile_index())
            .finish()
    }
}

impl fmt::Display for TileId {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}/{}", self.level(), self.tile_index())
    }
}

/// Represents errors returned by the Valhalla C++ API.
#[derive(Debug, Clone, PartialEq)]
pub struct Error(Box<str>);

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Error {}

impl From<cxx::Exception> for Error {
    fn from(err: cxx::Exception) -> Self {
        Error(err.what().into())
    }
}

bitflags! {
    /// Access bit field constants. Access in directed edge allows 12 bits.
    ///
    /// Valhalla's [costing models] decide accessibility with these bits: a travel mode may use an
    /// edge or a node if any of its bits is set in [`DirectedEdge::forwardaccess()`] or
    /// [`NodeInfo::access()`].
    ///
    /// [costing models]: https://valhalla.github.io/valhalla/api/turn-by-turn/api-reference/#costing-models
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct Access: u16 {
        const AUTO = 1;
        const PEDESTRIAN = 2;
        const BICYCLE = 4;
        const TRUCK = 8;
        const EMERGENCY = 16;
        const TAXI = 32;
        const BUS = 64;
        /// High-Occupancy Vehicle, i.e. carpool lanes marked via `hov=designated` or similar tags.
        const HOV = 128;
        const WHEELCHAIR = 256;
        const MOPED = 512;
        const MOTORCYCLE = 1024;
        const ALL = 4095;
        const VEHICULAR = Self::AUTO.bits() | Self::TRUCK.bits() | Self::MOPED.bits() | Self::MOTORCYCLE.bits()
                        | Self::TAXI.bits() | Self::BUS.bits() | Self::HOV.bits();
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct SpeedSources: u8 {
        /// Default edge speed - speed limit if available, otherwise typical speed for the edge type.
        const NO_FLOW = 0;
        /// Typical (average historical) speed during the night, from 7pm to 7am.
        const FREE_FLOW = 1;
        /// Typical (average historical) speed during the day, from 7am to 7pm.
        const CONSTRAINED_FLOW = 2;
        /// Historical traffic speed, stored in 5m buckets over the week.
        const PREDICTED_FLOW = 4;
        /// Live-traffic speed.
        const CURRENT_FLOW = 8;
        /// All available speed sources.
        const ALL = Self::FREE_FLOW.bits() | Self::CONSTRAINED_FLOW.bits()
                  | Self::PREDICTED_FLOW.bits() | Self::CURRENT_FLOW.bits();
    }
}

/// Coordinate in (lat, lon) format.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LatLon(pub f64, pub f64);

impl From<LatLon> for (f64, f64) {
    fn from(loc: LatLon) -> Self {
        (loc.0, loc.1)
    }
}

impl From<(f64, f64)> for LatLon {
    fn from((lat, lon): (f64, f64)) -> Self {
        LatLon(lat, lon)
    }
}

#[cfg(feature = "proto")]
impl From<LatLon> for proto::LatLng {
    fn from(loc: LatLon) -> Self {
        proto::LatLng {
            has_lat: Some(proto::lat_lng::HasLat::Lat(loc.0)),
            has_lng: Some(proto::lat_lng::HasLng::Lng(loc.1)),
        }
    }
}

/// Handy wrapper as [`proto::Location`] has optional `ll` field that actually always should be set.
#[cfg(feature = "proto")]
impl From<LatLon> for Option<proto::LatLng> {
    fn from(loc: LatLon) -> Self {
        Some(loc.into())
    }
}

/// High-level interface for reading Valhalla graph tiles from tar extracts.
///
/// As `GraphReader` already uses shared ownership internally, cloning is cheap and it can be
/// reused across threads without wrapping it in an [`Arc`].
///
/// Prefer cloning over creating another `GraphReader` from the same config, as each one memory-maps
/// the whole tileset.
#[derive(Clone)]
pub struct GraphReader(cxx::SharedPtr<ffi::TileSet>);

impl GraphReader {
    /// Creates a new GraphReader from the given Valhalla configuration, parsed into a [`Config`].
    ///
    /// # Examples
    ///
    /// ```
    /// let config = valhalla::ConfigBuilder {
    ///     mjolnir: valhalla::config::Mjolnir {
    ///         tile_extract: "path/to/tiles.tar".into(),
    ///         traffic_extract: "path/to/traffic.tar".into(), // optional
    ///         ..Default::default()
    ///     },
    ///     ..Default::default()
    /// }
    /// .build();
    /// let reader = valhalla::GraphReader::new(&config);
    /// ```
    pub fn new(config: &Config) -> Result<Self, Error> {
        Ok(Self(ffi::new_tileset(config.inner())?))
    }

    /// Latest OSM changeset ID (or the maximum OSM Node/Way/Relation ID) in the OSM PBF file used to build the tileset.
    pub fn dataset_id(&self) -> u64 {
        self.0.dataset_id()
    }

    /// List all tiles in the tileset.
    pub fn tiles(&self) -> Vec<TileId> {
        self.0.tiles()
    }

    /// List all tiles in the bounding box for a given hierarchy level in the tileset.
    pub fn tiles_in_bbox(&self, min: LatLon, max: LatLon, level: u8) -> Vec<TileId> {
        self.0.tiles_in_bbox(
            min.0 as f32,
            min.1 as f32,
            max.0 as f32,
            max.1 as f32,
            level,
        )
    }

    /// Retrieves the graph tile if it exists in the tileset. Takes a [`TileId`], or the [`GraphId`]
    /// of anything inside the tile.
    pub fn graph_tile(&self, id: impl Into<TileId>) -> Option<GraphTile> {
        GraphTile::new(self.0.get_graph_tile(id.into()))
    }

    /// Retrieves the live traffic tile if it exists in the tileset. Takes a [`TileId`], or the
    /// [`GraphId`] of anything inside the tile.
    pub fn traffic_tile(&self, id: impl Into<TileId>) -> Option<ffi::TrafficTile> {
        self.0.get_traffic_tile(id.into()).ok()
    }
}

/// Graph information for a tile within the Tiled Hierarchical Graph.
///
/// `GraphTile` uses manual reference counting via `boost::intrusive_ptr<T>` on the C++ side.
/// Cloning is cheap as it only increments the reference count.
///
/// It's not `Send`/`Sync` as the reference count isn't atomic, so call [`GraphReader::graph_tile()`]
/// from each thread instead of sharing instances.
///
/// `GraphTile` can outlive the [`GraphReader`] that created it.
pub struct GraphTile(NonNull<ffi::GraphTile>);

impl Clone for GraphTile {
    fn clone(&self) -> Self {
        unsafe { ffi::add_ref(self.0.as_ptr()) };
        Self(self.0)
    }
}

impl Drop for GraphTile {
    fn drop(&mut self) {
        unsafe { ffi::release(self.0.as_ptr()) };
    }
}

impl GraphTile {
    fn new(tile: *const ffi::GraphTile) -> Option<Self> {
        NonNull::new(tile.cast_mut()).map(Self)
    }

    /// Like `Deref`, but keeps the ffi methods private.
    fn inner(&self) -> &ffi::GraphTile {
        // Safety: the pointer comes from [`GraphTile::new()`] and the tile outlives `self`.
        unsafe { self.0.as_ref() }
    }

    /// Id of this tile.
    #[inline(always)]
    pub fn id(&self) -> TileId {
        self.inner().id().tile()
    }

    /// Slice of all directed edges in the current tile.
    #[inline(always)]
    pub fn directededges(&self) -> &[ffi::DirectedEdge] {
        ffi::directededges(self.inner())
    }

    /// Gets a directed edge by index within the current tile.
    pub fn directededge(&self, index: u32) -> Option<&ffi::DirectedEdge> {
        match self.inner().directededge(index as usize) {
            Ok(ptr) if !ptr.is_null() => Some(unsafe { &*ptr }),
            _ => None,
        }
    }

    /// Slice of all nodes in the current tile.
    #[inline(always)]
    pub fn nodes(&self) -> &[ffi::NodeInfo] {
        ffi::nodes(self.inner())
    }

    /// Gets a node by index within the current tile.
    pub fn node(&self, index: u32) -> Option<&ffi::NodeInfo> {
        match self.inner().node(index as usize) {
            Ok(ptr) if !ptr.is_null() => Some(unsafe { &*ptr }),
            _ => None,
        }
    }

    /// Coordinate in (lat,lon) format for the given node.
    /// This gives the exact location of the node with better precision than [`EdgeInfo::shape`] start/end points.
    #[inline(always)]
    pub fn node_latlon(&self, node: &ffi::NodeInfo) -> LatLon {
        debug_assert!(ref_within_slice(self.nodes(), node), "Wrong tile");
        let latlon = ffi::node_latlon(self.inner(), node);
        LatLon(latlon.lat, latlon.lon)
    }

    /// Slice of all outbound edges for the given node.
    #[inline(always)]
    pub fn node_edges<'a>(&'a self, node: &ffi::NodeInfo) -> &'a [ffi::DirectedEdge] {
        debug_assert!(ref_within_slice(self.nodes(), node), "Wrong tile");
        ffi::node_edges(self.inner(), node)
    }

    /// Slice of all transitions to other hierarchy levels for the given node.
    #[inline(always)]
    pub fn node_transitions<'a>(&'a self, node: &ffi::NodeInfo) -> &'a [ffi::NodeTransition] {
        debug_assert!(ref_within_slice(self.nodes(), node), "Wrong tile");
        ffi::node_transitions(self.inner(), node)
    }

    /// Information about the administrative area, such as country or state, by its index.
    /// Indices are stored in [`NodeInfo::admin_index()`] fields.
    pub fn admin_info(&self, index: u32) -> Option<ffi::AdminInfo> {
        ffi::admininfo(self.inner(), index).ok()
    }

    /// Dynamic (cold) information about the edge, such as OSM Way ID, speed limit, shape, elevation, etc.
    #[inline(always)]
    pub fn edgeinfo<'a>(&'a self, de: &ffi::DirectedEdge) -> EdgeInfo<'a> {
        debug_assert!(ref_within_slice(self.directededges(), de), "Wrong tile");
        ffi::edgeinfo(self.inner(), de)
    }

    /// Live traffic record for this edge. When no traffic is loaded the record carries no reading
    /// ([`LiveTraffic::speed()`] returns `None`). Interpret via [`LiveTraffic::speed`] and [`LiveTraffic::segments`].
    #[inline(always)]
    pub fn live_traffic(&self, de: &ffi::DirectedEdge) -> LiveTraffic {
        debug_assert!(ref_within_slice(self.directededges(), de), "Wrong tile");
        LiveTraffic::from_bits(ffi::live_traffic(self.inner(), de))
    }

    /// Overall edge speed in km/h, mixed from the requested [`SpeedSources`], along with the sources
    /// that were actually available for the edge.
    ///
    /// This function never returns zero speed, even if the edge is closed due to traffic. Read the edge's live
    /// traffic via [`GraphTile::live_traffic()`] and check [`LiveTraffic::speed()`] for
    /// `Some(0)` to determine if the edge is closed instead.
    pub fn edge_speed(
        &self,
        de: &ffi::DirectedEdge,
        speed_sources: SpeedSources,
        is_truck: bool,
        second_of_week: u64,
        seconds_from_now: u64,
    ) -> (u32, SpeedSources) {
        debug_assert!(ref_within_slice(self.directededges(), de), "Wrong tile");
        let mut flow_sources: u8 = 0;
        let speed = unsafe {
            self.inner().GetSpeed(
                de as *const ffi::DirectedEdge,
                speed_sources.bits(),
                second_of_week,
                is_truck,
                &mut flow_sources,
                seconds_from_now,
            )
        };
        (speed, SpeedSources::from_bits_retain(flow_sources))
    }
}

/// Directed edge within the graph.
#[repr(C)]
pub struct DirectedEdge {
    // Same size as the C++ type, so that slices of it index correctly.
    data: [u64; 6],
}

unsafe impl ExternType for DirectedEdge {
    type Id = cxx::type_id!("valhalla::baldr::DirectedEdge");
    type Kind = cxx::kind::Trivial;
}

impl DirectedEdge {
    /// Access modes in the forward direction. Bit mask using [`Access`] constants.
    #[inline(always)]
    pub fn forwardaccess(&self) -> Access {
        Access::from_bits_retain(self.forwardaccess_u32() as u16)
    }

    /// Access modes in the reverse direction. Bit mask using [`Access`] constants.
    #[inline(always)]
    pub fn reverseaccess(&self) -> Access {
        Access::from_bits_retain(self.reverseaccess_u32() as u16)
    }
}

/// Information held for each node within the graph. The graph uses a forward star structure:
/// nodes point to the first outbound directed edge and each directed edge points to the other
/// end node of the edge.
#[repr(C)]
pub struct NodeInfo {
    // Same size as the C++ type, so that slices of it index correctly.
    data: [u64; 4],
}

unsafe impl ExternType for NodeInfo {
    type Id = cxx::type_id!("valhalla::baldr::NodeInfo");
    type Kind = cxx::kind::Trivial;
}

impl NodeInfo {
    /// Access modes allowed to pass through the node. Bit mask using [`crate::Access`] constants.
    #[inline(always)]
    pub fn access(&self) -> Access {
        Access::from_bits_retain(self.access_u16())
    }
}

/// Records a transition between a node on the current tile and a node
/// at the same position on a different hierarchy level. Stores the GraphId
/// of the end node as well as a flag indicating whether the transition is
/// upwards (true) or downwards (false).
#[repr(C)]
pub struct NodeTransition {
    data: [u64; 1],
}

unsafe impl ExternType for NodeTransition {
    type Id = cxx::type_id!("valhalla::baldr::NodeTransition");
    type Kind = cxx::kind::Trivial;
}

impl TimeZoneInfo {
    /// Retrieves the timezone information by its index if available. `unix_timestamp` is required to handle DST.
    #[inline(always)]
    pub fn from_id(id: u32, unix_timestamp: u64) -> Option<Self> {
        ffi::from_id(id, unix_timestamp).ok()
    }
}

impl<'a> EdgeInfo<'a> {
    /// Shape points, decoded from the tile without allocating.
    #[inline(always)]
    pub fn shape(&self) -> Shape<'a> {
        Shape::new(self.encoded_shape)
    }
}

/// Checks if the given reference points to an item within the given slice.
fn ref_within_slice<T>(slice: &[T], item: &T) -> bool {
    slice.as_ptr_range().contains(&(item as *const T))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn graph_id() {
        let id = GraphId::new(5411833275938);
        assert_eq!(id.level(), 2);
        assert_eq!(id.tile_index(), 838852);
        assert_eq!(id.id(), 161285);
        assert_eq!(
            GraphId::from_parts(id.level(), id.tile_index(), id.id()),
            Some(id)
        );
        assert_eq!(format!("{id}"), "2/838852/161285");
        assert_eq!(
            format!("{id:?}"),
            "GraphId { level: 2, tile_index: 838852, id: 161285 }"
        );

        let default_id = GraphId::default();
        assert_eq!(default_id.level(), 7);
        assert_eq!(default_id.tile_index(), 4194303);
        assert_eq!(default_id.id(), 2097151);

        assert_eq!(GraphId::from_parts(8, id.tile_index(), 0), None);
    }

    #[test]
    fn tile_id() {
        let id = GraphId::new(5411833275938);
        let tile = id.tile();
        assert_eq!(tile, TileId::from(id));
        assert_eq!(tile.level(), 2);
        assert_eq!(tile.tile_index(), 838852);
        assert_eq!(format!("{tile}"), "2/838852");
        assert_eq!(
            format!("{tile:?}"),
            "TileId { level: 2, tile_index: 838852 }"
        );

        // Every id in the tile shares it, whatever its own id.
        assert_eq!(tile.graph_id(id.id()), Some(id));
        assert_eq!(tile.graph_id(0).map(|base| base.tile()), Some(tile));
        assert_eq!(tile.graph_id(0).map(|base| base.id()), Some(0));
        assert_eq!(tile.graph_id(1 << 21), None);

        // Same packing as C++ `GraphId::tile_value()`.
        assert_eq!(tile.value, (id.value & 0x1ffffff) as u32);

        let invalid = TileId::default();
        assert_eq!(invalid.level(), 7);
        assert_eq!(invalid.tile_index(), 4194303);
    }

    #[test]
    fn test_ref_within_slice() {
        let data = [10, 20, 30, 40, 50];
        assert!(ref_within_slice(&data, &data[0]));
        assert!(ref_within_slice(&data, &data[2]));
        assert!(ref_within_slice(&data, &data[4]));

        let outside = 30;
        assert!(!ref_within_slice(&data, &outside));

        let subslice = &data[1..4];
        assert!(!ref_within_slice(subslice, &data[0]));
        assert!(ref_within_slice(subslice, &data[1]));
        assert!(ref_within_slice(subslice, &data[2]));
        assert!(ref_within_slice(subslice, &data[3]));
        assert!(!ref_within_slice(subslice, &data[4]));
    }
}
