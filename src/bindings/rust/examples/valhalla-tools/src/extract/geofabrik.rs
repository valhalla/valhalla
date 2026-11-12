use std::collections::BTreeMap;

use serde_json::Value;
use tracing::info;

use super::Error;
use crate::geojson;
use crate::geometry::Polygon;

pub const GEOFABRIK_INDEX_URL: &str = "https://download.geofabrik.de/index-v1.json";

fn fetch_index(url: &str) -> Result<Value, Error> {
    info!("Fetching Geofabrik index from {url}...");
    let response = crate::http::agent()
        .get(url)
        .call()
        .map_err(|error| Error::Download {
            url: url.to_owned(),
            reason: Box::new(error),
        })?;
    let status = response.status();
    if !status.is_success() {
        return Err(Error::HttpStatus {
            url: url.to_owned(),
            status: status.as_u16(),
            reason: status.canonical_reason().unwrap_or_default(),
        });
    }
    serde_json::from_reader(response.into_body().into_reader()).map_err(|source| {
        Error::InvalidIndex {
            url: url.to_owned(),
            source,
        }
    })
}

fn features(index: &Value) -> &[Value] {
    index["features"].as_array().map_or(&[], Vec::as_slice)
}

fn property<'a>(feature: &'a Value, key: &str) -> &'a str {
    feature["properties"][key].as_str().unwrap_or_default()
}

/// Finds a region by id or name, ignoring case, or by its `parent/id` path.
fn find_region<'a>(index: &'a Value, query: &str) -> Option<&'a Value> {
    let query = query.to_lowercase();
    features(index).iter().find(|feature| {
        let id = property(feature, "id").to_lowercase();
        let name = property(feature, "name").to_lowercase();
        let parent = property(feature, "parent").to_lowercase();
        query == id || query == name || (query.contains('/') && query == format!("{parent}/{id}"))
    })
}

pub(crate) fn region_polygons(url: &str, regions: &[String]) -> Result<Vec<Polygon>, Error> {
    let index = fetch_index(url)?;
    let mut polygons = Vec::new();
    for region in regions {
        let feature =
            find_region(&index, region).ok_or_else(|| Error::UnknownRegion(region.clone()))?;
        info!(
            "Using boundary for '{}' ({})",
            property(feature, "name"),
            property(feature, "id")
        );
        match geojson::polygons(&feature["geometry"]) {
            Some(region_polygons) if !region_polygons.is_empty() => {
                polygons.extend(region_polygons);
            }
            _ => return Err(Error::InvalidRegionGeometry(region.clone())),
        }
    }
    Ok(polygons)
}

/// The Geofabrik regions grouped by parent, then sorted by name.
pub fn region_list(url: &str) -> Result<String, Error> {
    Ok(format_region_list(&fetch_index(url)?))
}

fn format_region_list(index: &Value) -> String {
    let mut by_parent: BTreeMap<&str, Vec<(&str, &str)>> = BTreeMap::new();
    for feature in features(index) {
        by_parent
            .entry(property(feature, "parent"))
            .or_default()
            .push((property(feature, "name"), property(feature, "id")));
    }

    let mut listing = String::new();
    for (parent, mut regions) in by_parent {
        let header = if parent.is_empty() {
            "(top-level)"
        } else {
            parent
        };
        listing.push_str(&format!("\n{header}:\n"));
        regions.sort_by_key(|&(name, _)| name);
        for (name, id) in regions {
            listing.push_str(&format!("  {name:40} {id}\n"));
        }
    }
    listing
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;

    fn index() -> Value {
        json!({
            "type": "FeatureCollection",
            "features": [
                {
                    "properties": {"id": "europe", "name": "Europe", "parent": ""},
                    "geometry": {"type": "MultiPolygon", "coordinates": [[[[-25, 35], [45, 35], [45, 72], [-25, 35]]]]}
                },
                {
                    "properties": {"id": "switzerland", "name": "Switzerland", "parent": "europe"},
                    "geometry": {"type": "MultiPolygon", "coordinates": [[[[5.9, 45.8], [10.5, 45.8], [10.5, 47.8], [5.9, 45.8]]]]}
                },
                {
                    "properties": {"id": "france", "name": "France", "parent": "europe"},
                    "geometry": {"type": "Polygon", "coordinates": [[[-5.1, 42.3], [8.2, 42.3], [8.2, 51.1], [-5.1, 42.3]]]}
                }
            ]
        })
    }

    fn found_id(query: &str) -> Option<String> {
        let index = index();
        find_region(&index, query).map(|feature| property(feature, "id").to_owned())
    }

    #[test]
    fn a_region_is_found_by_id() {
        assert_eq!(found_id("switzerland").as_deref(), Some("switzerland"));
    }

    #[test]
    fn a_region_is_found_by_name() {
        assert_eq!(found_id("France").as_deref(), Some("france"));
    }

    #[test]
    fn a_region_name_matches_regardless_of_case() {
        assert_eq!(found_id("SWITZERLAND").as_deref(), Some("switzerland"));
    }

    #[test]
    fn a_region_is_found_by_its_parent_path() {
        assert_eq!(
            found_id("europe/switzerland").as_deref(),
            Some("switzerland")
        );
    }

    #[test]
    fn a_wrong_parent_path_finds_nothing() {
        assert_eq!(found_id("asia/switzerland"), None);
    }

    #[test]
    fn an_unknown_region_finds_nothing() {
        assert_eq!(found_id("narnia"), None);
    }

    #[test]
    fn regions_are_listed_by_parent_then_name() {
        let expected = format!(
            "\n(top-level):\n  {:40} europe\n\neurope:\n  {:40} france\n  {:40} switzerland\n",
            "Europe", "France", "Switzerland"
        );
        assert_eq!(format_region_list(&index()), expected);
    }
}
