//! Polygons of GeoJSON files, as the scripts read them with shapely.

use std::fs;
use std::io;
use std::path::{Path, PathBuf};

use serde_json::Value;

use crate::geometry::{Point, Polygon};
use crate::paths::resolved;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("Geojson directory does not exist or contains no GeoJSON files: {}", .0.display())]
    NoFiles(PathBuf),
    #[error("{}: expected a FeatureCollection with valid Polygon or MultiPolygon geometries", .0.display())]
    Invalid(PathBuf),
    #[error("I/O error on {}", path.display())]
    Io {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("invalid JSON in {}", path.display())]
    Json {
        path: PathBuf,
        #[source]
        source: serde_json::Error,
    },
}

/// Exterior rings of a `Polygon` or `MultiPolygon` geometry; other geometry types have none.
/// Returns `None` when the coordinates are malformed.
pub(crate) fn polygons(geometry: &Value) -> Option<Vec<Polygon>> {
    let coordinates = &geometry["coordinates"];
    match geometry["type"].as_str() {
        Some("Polygon") => Some(vec![exterior_ring(coordinates)?]),
        Some("MultiPolygon") => coordinates.as_array()?.iter().map(exterior_ring).collect(),
        _ => Some(Vec::new()),
    }
}

fn exterior_ring(rings: &Value) -> Option<Polygon> {
    let ring = rings.as_array()?.first()?.as_array()?;
    let points = ring
        .iter()
        .map(|position| {
            Some(Point {
                x: position.get(0)?.as_f64()?,
                y: position.get(1)?.as_f64()?,
            })
        })
        .collect::<Option<Vec<_>>>()?;
    Some(Polygon::new(points))
}

/// Polygons of every feature in the `*.geojson` files directly inside `dir`.
pub(crate) fn read_dir(dir: &Path) -> Result<Vec<Polygon>, Error> {
    let entries = fs::read_dir(dir).map_err(|_| Error::NoFiles(resolved(dir)))?;
    let mut files = Vec::new();
    for entry in entries {
        let path = entry
            .map_err(|source| Error::Io {
                path: dir.to_path_buf(),
                source,
            })?
            .path();
        if path
            .extension()
            .is_some_and(|extension| extension == "geojson")
        {
            files.push(path);
        }
    }
    if files.is_empty() {
        return Err(Error::NoFiles(resolved(dir)));
    }

    let mut polygons = Vec::new();
    for path in files {
        polygons.extend(read_file(path)?);
    }
    Ok(polygons)
}

fn read_file(path: PathBuf) -> Result<Vec<Polygon>, Error> {
    let text = match fs::read_to_string(&path) {
        Ok(text) => text,
        Err(source) => return Err(Error::Io { path, source }),
    };
    let collection: Value = match serde_json::from_str(&text) {
        Ok(collection) => collection,
        Err(source) => return Err(Error::Json { path, source }),
    };
    let Some(features) = collection["features"].as_array() else {
        return Err(Error::Invalid(path));
    };
    let mut file_polygons = Vec::new();
    for feature in features {
        match polygons(&feature["geometry"]) {
            Some(feature_polygons) => file_polygons.extend(feature_polygons),
            None => return Err(Error::Invalid(path)),
        }
    }
    Ok(file_polygons)
}

#[cfg(test)]
#[expect(clippy::unwrap_used, reason = "tests fail loudly on bad fixtures")]
mod tests {
    use serde_json::json;

    use super::*;

    fn square(min: f64, max: f64) -> Value {
        json!([[min, min], [max, min], [max, max], [min, max], [min, min]])
    }

    #[test]
    fn a_polygon_keeps_only_its_exterior_ring() {
        let geometry =
            json!({"type": "Polygon", "coordinates": [square(0.0, 4.0), square(1.0, 2.0)]});
        let expected = polygons(&json!({"type": "Polygon", "coordinates": [square(0.0, 4.0)]}));
        assert_eq!(polygons(&geometry), expected);
    }

    #[test]
    fn a_multipolygon_gives_one_polygon_per_part() {
        let geometry = json!({
            "type": "MultiPolygon",
            "coordinates": [[square(0.0, 1.0), square(0.2, 0.4)], [square(5.0, 6.0)]]
        });
        assert_eq!(polygons(&geometry).unwrap().len(), 2);
    }

    #[test]
    fn positions_with_an_elevation_are_accepted() {
        let geometry =
            json!({"type": "Polygon", "coordinates": [[[0, 0, 10], [1, 0, 10], [0, 1, 10]]]});
        assert_eq!(polygons(&geometry).unwrap().len(), 1);
    }

    #[test]
    fn other_geometry_types_give_no_polygon() {
        let geometry = json!({"type": "Point", "coordinates": [0, 0]});
        assert_eq!(polygons(&geometry), Some(Vec::new()));
    }

    #[test]
    fn a_polygon_without_rings_is_malformed() {
        let geometry = json!({"type": "Polygon", "coordinates": []});
        assert_eq!(polygons(&geometry), None);
    }

    #[test]
    fn a_position_with_text_is_malformed() {
        let geometry = json!({"type": "Polygon", "coordinates": [[[0, 0], ["1", 0], [0, 1]]]});
        assert_eq!(polygons(&geometry), None);
    }
}
