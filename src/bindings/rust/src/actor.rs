use prost::Message;

use crate::{Config, Error, proto, proto::options::Format};

#[allow(clippy::needless_lifetimes)] // clippy goes nuts with cxx
#[cxx::bridge]
mod ffi {
    /// Helper struct to provide an access to C++'s buffer with serialized response data.
    struct Response {
        /// Raw response data, either a JSON string or binary data.
        data: UniquePtr<CxxString>,
        /// [`options::Format`] format of the response.
        format: i32,
    }

    unsafe extern "C++" {
        include!("valhalla/src/actor.hpp");

        #[namespace = "boost::property_tree"]
        type ptree = crate::config::ffi::ptree;

        type Actor;
        fn new_actor(config: &ptree) -> Result<UniquePtr<Actor>>;
        // All methods accept [`proto::Options`] object serialized as a byte slice.
        fn route(self: Pin<&mut Actor>, request: &[u8]) -> Result<Response>;
        fn locate(self: Pin<&mut Actor>, request: &[u8]) -> Result<Response>;
        fn matrix(self: Pin<&mut Actor>, request: &[u8]) -> Result<Response>;
        fn optimized_route(self: Pin<&mut Actor>, request: &[u8]) -> Result<Response>;
        fn isochrone(self: Pin<&mut Actor>, request: &[u8]) -> Result<Response>;
        fn trace_route(self: Pin<&mut Actor>, request: &[u8]) -> Result<Response>;
        fn trace_attributes(self: Pin<&mut Actor>, request: &[u8]) -> Result<Response>;
        fn transit_available(self: Pin<&mut Actor>, request: &[u8]) -> Result<Response>;
        fn expansion(self: Pin<&mut Actor>, request: &[u8]) -> Result<Response>;
        fn centroid(self: Pin<&mut Actor>, request: &[u8]) -> Result<Response>;
        fn status(self: Pin<&mut Actor>, request: &[u8]) -> Result<Response>;

        /// Returns [`proto::Options`] object serialized as C++ `std::string` from a Valhalla JSON string.
        fn parse_json_request(json: &str, action: i32) -> Result<UniquePtr<CxxString>>;
    }
}

// Safety: `ffi::Actor` doesn't hold any reference to the shared state and all its methods require
// a mutable reference to `self`, so stronger borrowing rules apply here, preventing using mutable
// methods concurrently.
unsafe impl Send for ffi::Actor {}
unsafe impl Sync for ffi::Actor {}

/// Valhalla natively supports multiple response formats, such as JSON, OSRM-like JSON, PBF, and others.
/// This format is specified on per-request basis using [`proto::Options`] `format` field, selecting one of the
/// [`proto::options::Format`] options.
///
/// It is worth noting that at the moment not every endpoint supports all of the formats. For example,
/// [`Actor::locate()`] will always return a Valhalla JSON response regardless of the `format` field in the request.
#[derive(Debug, Clone)]
pub enum Response {
    /// Protocol Buffer response containing a structured [`proto::Api`] object.
    ///
    /// This format preserves full type information and is most efficient for
    /// programmatic access. Only available when [`proto::options::Format::Pbf`]
    /// is explicitly requested and the endpoint supports it.
    ///
    /// Set [`proto::PbfFieldSelector`] request option to get more detailed output.
    Pbf(Box<proto::Api>),
    /// JSON string response in either Valhalla or OSRM format.
    ///
    /// The JSON structure depends on which format was requested:
    /// - [`proto::options::Format::Json`] returns Valhalla-native JSON
    /// - [`proto::options::Format::Osrm`] returns OSRM-compatible JSON
    ///
    /// This is also the fallback format for most endpoints when other formats
    /// are not supported, like in the case of [`Actor::locate()`] that always
    /// returns a Valhalla JSON response.
    Json(String),
    /// Binary response for specialized formats.
    ///
    /// Currently used for:
    /// - GPX files (XML format for GPS data)
    /// - GeoTIFF files (raster geographic data)
    ///
    /// The actual format depends on the request and endpoint capabilities.
    Other(Vec<u8>),
}

impl From<ffi::Response> for Response {
    fn from(response: ffi::Response) -> Self {
        if response.format == Format::Pbf as i32 {
            let api = proto::Api::decode(response.data.as_bytes())
                .expect("Proper PBF data is guaranteed by Valhalla");
            Response::Pbf(Box::new(api))
        } else if response.format == Format::Json as i32 || response.format == Format::Osrm as i32 {
            Response::Json(String::from_utf8_lossy(response.data.as_bytes()).into_owned())
        } else {
            Response::Other(response.data.as_bytes().to_owned())
        }
    }
}

/// High-level interface to interact with [Valhalla's API](https://valhalla.github.io/valhalla/api/).
/// On contrary to the Valhalla REST and C++ APIs, this interface is designed to be used with [`proto::Options`] only,
/// to avoid unnecessary conversions and to provide a strongly typed interface.
pub struct Actor(cxx::UniquePtr<ffi::Actor>);

impl Actor {
    /// ```
    /// let config = valhalla::ConfigBuilder {
    ///     mjolnir: valhalla::config::Mjolnir {
    ///         tile_extract: "path/to/tiles.tar".into(),
    ///         traffic_extract: "path/to/traffic.tar".into(),
    ///         ..Default::default()
    ///     },
    ///     ..Default::default()
    /// }
    /// .build();
    /// let actor = valhalla::Actor::new(&config);
    /// ```
    pub fn new(config: &Config) -> Result<Self, Error> {
        Ok(Self(ffi::new_actor(config.inner())?))
    }

    /// Calculates a route between locations.
    ///
    /// # Example
    ///
    /// ```
    /// # fn call_route(actor: &mut valhalla::Actor) {
    /// use valhalla::proto;
    ///
    /// let request = proto::Options {
    ///     format: proto::options::Format::Pbf as i32,
    ///     costing_type: proto::costing::Type::Auto as i32,
    ///     locations: vec![
    ///         proto::Location {
    ///              ll: valhalla::LatLon(55.6086, 13.0005).into(),
    ///              ..Default::default()
    ///          },
    ///          proto::Location {
    ///              ll: valhalla::LatLon(55.5944, 13.0002).into(),
    ///              ..Default::default()
    ///          },
    ///     ],
    ///     ..Default::default()
    /// };
    /// let response = actor.route(&request);
    /// let Ok(valhalla::Response::Pbf(api)) = response else {
    ///     panic!("Expected PBF response, got: {response:?}");
    /// };
    /// # }
    /// ```
    pub fn route(&mut self, request: &proto::Options) -> Result<Response, Error> {
        self.act(ffi::Actor::route, request)
    }

    /// Finds the nearest roads and intersections to input coordinates. Always returns a Valhalla JSON response.
    ///
    /// # Examples
    ///
    /// ```
    /// # fn call_locate(mut actor: valhalla::Actor) {
    /// use valhalla::proto;
    ///
    /// let request = proto::Options {
    ///     locations: vec![
    ///         proto::Location {
    ///             ll: valhalla::LatLon(55.6086, 13.0005).into(),
    ///             ..Default::default()
    ///         },
    ///     ],
    ///     has_verbose: Some(proto::options::HasVerbose::Verbose(true)),
    ///     ..Default::default()
    /// };
    /// let response = actor.locate(&request);
    /// let Ok(valhalla::Response::Json(json)) = response else {
    ///     panic!("Expected JSON response, got: {response:?}");
    /// };
    /// # }
    /// ```
    pub fn locate(&mut self, request: &proto::Options) -> Result<Response, Error> {
        self.act(ffi::Actor::locate, request)
    }

    /// Computes a time-distance matrix between sources and targets.
    ///
    /// # Examples
    ///
    /// ```
    /// # fn call_matrix(mut actor: valhalla::Actor) {
    /// use valhalla::proto;
    ///
    /// let request = proto::Options {
    ///     costing_type: proto::costing::Type::Auto as i32,
    ///     sources: vec![
    ///         proto::Location {
    ///             ll: valhalla::LatLon(55.6086, 13.0005).into(),
    ///             ..Default::default()
    ///         },
    ///     ],
    ///     targets: vec![
    ///         proto::Location {
    ///             ll: valhalla::LatLon(55.5944, 13.0002).into(),
    ///             ..Default::default()
    ///         },
    ///     ],
    ///     ..Default::default()
    /// };
    /// let response = actor.matrix(&request);
    /// # }
    /// ```
    pub fn matrix(&mut self, request: &proto::Options) -> Result<Response, Error> {
        self.act(ffi::Actor::matrix, request)
    }

    /// Solves the traveling salesman problem for multiple locations.
    ///
    /// # Examples
    ///
    /// ```
    /// # fn call_optimized_route(mut actor: valhalla::Actor) {
    /// use valhalla::proto;
    ///
    /// let request = proto::Options {
    ///     costing_type: proto::costing::Type::Auto as i32,
    ///     locations: vec![
    ///         proto::Location {
    ///             ll: valhalla::LatLon(55.6086, 13.0005).into(),
    ///             ..Default::default()
    ///         },
    ///         proto::Location {
    ///             ll: valhalla::LatLon(55.5944, 13.0002).into(),
    ///             ..Default::default()
    ///         },
    ///         proto::Location {
    ///             ll: valhalla::LatLon(55.6000, 13.0050).into(),
    ///             ..Default::default()
    ///         },
    ///     ],
    ///     ..Default::default()
    /// };
    /// let response = actor.optimized_route(&request);
    /// # }
    /// ```
    pub fn optimized_route(&mut self, request: &proto::Options) -> Result<Response, Error> {
        self.act(ffi::Actor::optimized_route, request)
    }

    /// Computes areas reachable within specified time or distance intervals.
    ///
    /// # Examples
    ///
    /// ```
    /// # fn call_isochrone(mut actor: valhalla::Actor) {
    /// use valhalla::proto;
    ///
    /// let request = proto::Options {
    ///     costing_type: proto::costing::Type::Pedestrian as i32,
    ///     locations: vec![
    ///         proto::Location {
    ///             ll: valhalla::LatLon(55.6086, 13.0005).into(),
    ///             ..Default::default()
    ///         },
    ///     ],
    ///     contours: vec![
    ///         proto::Contour {
    ///             has_time: Some(proto::contour::HasTime::Time(10.0)),
    ///             ..Default::default()
    ///         },
    ///     ],
    ///     ..Default::default()
    /// };
    /// let response = actor.isochrone(&request);
    /// # }
    /// ```
    pub fn isochrone(&mut self, request: &proto::Options) -> Result<Response, Error> {
        self.act(ffi::Actor::isochrone, request)
    }

    /// Map-matches a GPS trace to roads and returns a route with turn-by-turn directions.
    ///
    /// # Examples
    ///
    /// ```
    /// # fn call_trace_route(mut actor: valhalla::Actor) {
    /// use valhalla::proto;
    ///
    /// let request = proto::Options {
    ///     costing_type: proto::costing::Type::Auto as i32,
    ///     has_encoded_polyline: Some(proto::options::HasEncodedPolyline::EncodedPolyline(
    ///         "_grbgAh~{nhF?lBAzBFvBHxBEtBKdB".into(),
    ///     )),
    ///     ..Default::default()
    /// };
    /// let response = actor.trace_route(&request).unwrap();
    /// # }
    /// ```
    pub fn trace_route(&mut self, request: &proto::Options) -> Result<Response, Error> {
        self.act(ffi::Actor::trace_route, request)
    }

    /// Map-matches a GPS trace and returns detailed edge attributes along the path.
    ///
    /// # Examples
    ///
    /// ```
    /// # fn call_trace_attributes(mut actor: valhalla::Actor) {
    /// use valhalla::proto;
    ///
    /// let request = proto::Options {
    ///     costing_type: proto::costing::Type::Auto as i32,
    ///     has_encoded_polyline: Some(proto::options::HasEncodedPolyline::EncodedPolyline(
    ///         "_grbgAh~{nhF?lBAzBFvBHxBEtBKdB".into(),
    ///     )),
    ///     ..Default::default()
    /// };
    /// let response = actor.trace_attributes(&request).unwrap();
    /// # }
    /// ```
    pub fn trace_attributes(&mut self, request: &proto::Options) -> Result<Response, Error> {
        self.act(ffi::Actor::trace_attributes, request)
    }

    /// Checks if transit/public transportation is available at given locations.
    ///
    /// # Examples
    ///
    /// ```
    /// # fn call_transit_available(mut actor: valhalla::Actor) {
    /// use valhalla::proto;
    ///
    /// let request = proto::Options {
    ///     locations: vec![
    ///         proto::Location {
    ///             ll: valhalla::LatLon(55.6086, 13.0005).into(),
    ///             ..Default::default()
    ///         },
    ///     ],
    ///     ..Default::default()
    /// };
    /// let response = actor.transit_available(&request).unwrap();
    /// # }
    /// ```
    pub fn transit_available(&mut self, request: &proto::Options) -> Result<Response, Error> {
        self.act(ffi::Actor::transit_available, request)
    }

    /// Returns a GeoJSON representation of graph traversal for visualization.
    ///
    /// # Examples
    ///
    /// ```
    /// # fn call_expansion(mut actor: valhalla::Actor) {
    /// use valhalla::proto;
    ///
    /// let request = proto::Options {
    ///     action: proto::options::Action::Route as i32,
    ///     has_expansion_action: Some(proto::options::HasExpansionAction::ExpansionAction(
    ///         proto::options::Action::Route as i32,
    ///     )),
    ///     costing_type: proto::costing::Type::Pedestrian as i32,
    ///     locations: vec![
    ///         proto::Location {
    ///             ll: valhalla::LatLon(55.6086, 13.0005).into(),
    ///             ..Default::default()
    ///         },
    ///         proto::Location {
    ///             ll: valhalla::LatLon(55.5944, 13.0002).into(),
    ///             ..Default::default()
    ///         },
    ///     ],
    ///     ..Default::default()
    /// };
    /// let response = actor.expansion(&request).unwrap();
    /// # }
    /// ```
    pub fn expansion(&mut self, request: &proto::Options) -> Result<Response, Error> {
        self.act(ffi::Actor::expansion, request)
    }

    /// Finds the least cost convergence point from multiple locations.
    ///
    /// # Examples
    ///
    /// ```
    /// # fn call_centroid(mut actor: valhalla::Actor) {
    /// use valhalla::proto;
    ///
    /// let request = proto::Options {
    ///     costing_type: proto::costing::Type::Auto as i32,
    ///     locations: vec![
    ///         proto::Location {
    ///             ll: valhalla::LatLon(55.6086, 13.0005).into(),
    ///             ..Default::default()
    ///         },
    ///         proto::Location {
    ///             ll: valhalla::LatLon(55.5944, 13.0002).into(),
    ///             ..Default::default()
    ///         },
    ///     ],
    ///     ..Default::default()
    /// };
    /// let response = actor.centroid(&request).unwrap();
    /// # }
    /// ```
    pub fn centroid(&mut self, request: &proto::Options) -> Result<Response, Error> {
        self.act(ffi::Actor::centroid, request)
    }

    /// Returns status information about the Valhalla instance and loaded tileset.
    ///
    /// # Examples
    ///
    /// ```
    /// # fn call_status(mut actor: valhalla::Actor) {
    /// use valhalla::proto;
    ///
    /// let request = proto::Options {
    ///     has_verbose: Some(proto::options::HasVerbose::Verbose(true)),
    ///     ..Default::default()
    /// };
    /// let response = actor.status(&request).unwrap();
    /// # }
    /// ```
    pub fn status(&mut self, request: &proto::Options) -> Result<Response, Error> {
        self.act(ffi::Actor::status, request)
    }

    /// Generic helper function to process request encoding, calling the endpoint and handling response.
    fn act<F>(&mut self, action_fn: F, request: &proto::Options) -> Result<Response, Error>
    where
        F: for<'a> Fn(
            std::pin::Pin<&'a mut ffi::Actor>,
            &'a [u8],
        ) -> Result<ffi::Response, cxx::Exception>,
    {
        let buffer = request.encode_to_vec();
        let result = action_fn(self.0.as_mut().unwrap(), &buffer);
        Ok(Response::from(result?))
    }

    /// Helper function to convert a Valhalla JSON string into Valhalla PBF request as [`proto::Options`] object.
    /// This function is not optimized for performance and should be considered as a convenience method.
    /// For best performance construct [`proto::Options`] directly if possible.
    pub fn parse_json_request(
        json: &str,
        action: proto::options::Action,
    ) -> Result<proto::Options, Error> {
        if json.is_empty() {
            // Empty string is a special for Valhalla, so we should return an error here.
            return Err(Error("Failed to parse json request".into()));
        }

        let cxx_string = ffi::parse_json_request(json, action as i32)?;
        let mut options = proto::Options::decode(cxx_string.as_bytes())
            .map_err(|err| Error(err.to_string().into()))?;

        // Workaround for "ignore_closures in costing and exclude_closures in search_filter cannot both be specified"
        // that is happened because this check is happens before that value is set to the default false and processing
        // json request actually causes parsing function to be called twice because it parses and sets default values.
        for costing in options.costings.values_mut() {
            if let Some(proto::costing::HasOptions::Options(costing_options)) =
                &mut costing.has_options
            {
                // If ignore_closures is false, we can clear it
                if let Some(proto::costing::options::HasIgnoreClosures::IgnoreClosures(false)) =
                    costing_options.has_ignore_closures
                {
                    costing_options.has_ignore_closures = None;
                }
            }
        }

        Ok(options)
    }
}
