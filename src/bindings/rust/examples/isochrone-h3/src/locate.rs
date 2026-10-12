//! Snapping a coordinate onto the graph with `Actor::locate`.
//!
//! `/locate` has no protobuf serializer and always answers with JSON, whatever `Options::format`
//! says — hence the serde structs below.

use anyhow::{Result, anyhow, bail};
use serde::{Deserialize, Deserializer};
use valhalla::{
    Actor, GraphId, LatLon,
    proto::{self, location::HasSearchCutoff::SearchCutoff, options::HasVerbose::Verbose},
};

/// Snaps `coord` onto the graph and returns the candidate edges, nearest first.
pub fn locate(actor: &mut Actor, coord: LatLon, search_radius: u32) -> Result<Vec<GraphId>> {
    let request = proto::Options {
        costing_type: proto::costing::Type::Auto as i32,
        locations: vec![proto::Location {
            ll: coord.into(),
            has_search_cutoff: Some(SearchCutoff(search_radius)),
            ..Default::default()
        }],
        has_verbose: Some(Verbose(true)),
        ..Default::default()
    };

    let response = actor.locate(&request);
    let Ok(valhalla::Response::Json(json)) = response else {
        return Err(anyhow!(
            "expected a JSON response from /locate, got {response:?}"
        ));
    };

    let located: Vec<Located> = serde_json::from_str(&json)
        .map_err(|e| anyhow!("failed to parse /locate response: {e}"))?;

    let edges = located.first().map(|l| l.edges.as_slice()).unwrap_or(&[]);
    if edges.is_empty() {
        bail!(
            "no edges within {search_radius}m of {:.5},{:.5} — is the coordinate inside the tileset?",
            coord.0,
            coord.1
        );
    }
    Ok(edges
        .iter()
        .map(|e| GraphId::new(e.edge_id.value))
        .collect())
}

/// Output of a Valhalla `/locate` request, trimmed to the one field we need.
/// <https://valhalla.github.io/valhalla/api/locate/api-reference/#outputs-of-a-locate-request>
#[derive(Deserialize)]
struct Located {
    #[serde(default, deserialize_with = "null_as_default")]
    edges: Vec<Edge>,
}

#[derive(Deserialize)]
struct Edge {
    edge_id: EdgeId,
}

#[derive(Deserialize)]
struct EdgeId {
    value: u64,
}

/// `/locate` writes `"edges": null` rather than omitting the field when nothing correlates.
/// Workaround for <https://github.com/serde-rs/serde/issues/1098>.
fn null_as_default<'de, D, T>(deserializer: D) -> Result<T, D::Error>
where
    T: Default + Deserialize<'de>,
    D: Deserializer<'de>,
{
    Ok(Option::deserialize(deserializer)?.unwrap_or_default())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn edge_ids(json: &str) -> Vec<u64> {
        let located: Vec<Located> = serde_json::from_str(json).unwrap();
        located[0].edges.iter().map(|e| e.edge_id.value).collect()
    }

    #[test]
    fn parses_correlated_edges() {
        let json = r#"[{"input_lat":42.5,"input_lon":1.5,"edges":[
            {"edge_id":{"value":1234},"correlated_lat":42.5},
            {"edge_id":{"value":5678},"correlated_lat":42.5}
        ]}]"#;
        assert_eq!(edge_ids(json), vec![1234, 5678]);
    }

    #[test]
    fn empty_edges_is_not_an_error() {
        assert!(edge_ids(r#"[{"input_lat":42.5,"edges":[]}]"#).is_empty());
    }

    #[test]
    fn null_edges_is_not_an_error() {
        assert!(edge_ids(r#"[{"input_lat":42.5,"edges":null}]"#).is_empty());
    }

    #[test]
    fn missing_edges_is_not_an_error() {
        assert!(edge_ids(r#"[{"input_lat":42.5}]"#).is_empty());
    }
}
