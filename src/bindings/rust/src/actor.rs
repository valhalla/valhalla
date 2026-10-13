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
        include!("valhalla/src/bindings/rust/src/actor.h");

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

/// Response of an [`Actor`] endpoint, in the format requested by the `format` field of
/// [`proto::Options`]. Not every endpoint supports every format, e.g. [`Actor::locate()`] always
/// returns JSON.
#[derive(Debug, Clone)]
pub enum Response {
    /// [`proto::options::Format::Pbf`]. Set [`proto::PbfFieldSelector`] for more detailed output.
    Pbf(Box<proto::Api>),
    /// Valhalla or OSRM JSON, also used when an endpoint doesn't support the requested format.
    Json(String),
    /// GPX or GeoTIFF.
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
/// Unlike Valhalla's HTTP and C++ APIs, it takes [`proto::Options`] only, which avoids conversions
/// and keeps the interface strongly typed.
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
    /// # Examples
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
    /// # fn call_locate(actor: &mut valhalla::Actor) {
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
    /// # fn call_matrix(actor: &mut valhalla::Actor) {
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
    /// # fn call_optimized_route(actor: &mut valhalla::Actor) {
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
    /// # fn call_isochrone(actor: &mut valhalla::Actor) {
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
    /// # fn call_trace_route(actor: &mut valhalla::Actor) {
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
    /// # fn call_trace_attributes(actor: &mut valhalla::Actor) {
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
    /// # fn call_transit_available(actor: &mut valhalla::Actor) {
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
    /// # fn call_expansion(actor: &mut valhalla::Actor) {
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
    /// # fn call_centroid(actor: &mut valhalla::Actor) {
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
    /// # fn call_status(actor: &mut valhalla::Actor) {
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

    /// Converts a Valhalla JSON request into [`proto::Options`]. It's a convenience method,
    /// building [`proto::Options`] directly is faster.
    pub fn parse_json_request(
        json: &str,
        action: proto::options::Action,
    ) -> Result<proto::Options, Error> {
        if json.is_empty() {
            // ParseApi treats an empty string as already parsed options
            return Err(Error("Failed to parse json request".into()));
        }

        let cxx_string = ffi::parse_json_request(json, action as i32)?;
        let mut options = proto::Options::decode(cxx_string.as_bytes())
            .map_err(|err| Error(err.to_string().into()))?;

        // ParseApi fills in `ignore_closures: false`, which the second ParseApi in `act()` rejects
        // when the request also has `exclude_closures`
        for costing in options.costings.values_mut() {
            if let Some(proto::costing::HasOptions::Options(costing_options)) =
                &mut costing.has_options
                && let Some(proto::costing::options::HasIgnoreClosures::IgnoreClosures(false)) =
                    costing_options.has_ignore_closures
            {
                costing_options.has_ignore_closures = None;
            }
        }

        Ok(options)
    }
}
