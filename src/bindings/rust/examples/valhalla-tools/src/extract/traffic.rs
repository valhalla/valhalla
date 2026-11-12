use std::fs::File;
use std::io::{self, Cursor, Read, Seek, SeekFrom};
use std::path::Path;

use tracing::debug;

use super::Error;
use super::archive::{self, ArchivedTile, Member, MemberMetadata, WriteMode};

/// `TRAFFIC_TILE_VERSION` in baldr/traffictile.h, which is the major version of Valhalla.
const TRAFFIC_TILE_VERSION: u32 = 3;
/// Size of `TrafficTileHeader`, followed by one 8-byte `TrafficSpeed` per directed edge.
const TRAFFIC_HEADER_SIZE: u64 = 32;
const TRAFFIC_SPEED_SIZE: u64 = 8;
/// Offset in `GraphTileHeader` of the 64-bit field packing the node and directed edge counts.
const RECORD_COUNTS_OFFSET: u64 = 40;

pub(crate) fn write_extract(
    tile_extract: &Path,
    tiles: &[ArchivedTile],
    output: &Path,
    mode: WriteMode,
) -> Result<(), Error> {
    let mut extract = File::open(tile_extract).map_err(Error::io(tile_extract))?;
    archive::write_indexed_tar(output, mode, tiles, |tile| {
        let edge_count =
            read_directed_edge_count(&mut extract, tile).map_err(|source| match source.kind() {
                io::ErrorKind::UnexpectedEof => Error::TruncatedTileHeader {
                    name: tile.name.clone(),
                },
                _ => Error::io(tile_extract)(source),
            })?;
        debug!("Traffic tile {} has {edge_count} directed edges", tile.name);
        let speeds_size = TRAFFIC_SPEED_SIZE * u64::from(edge_count);
        let header = traffic_header(tile.entry.tile_id, edge_count);
        Ok(Member {
            name: tile.name.clone(),
            tile_id: tile.entry.tile_id,
            size: TRAFFIC_HEADER_SIZE + speeds_size,
            metadata: MemberMetadata::created_now(),
            content: Cursor::new(header).chain(io::repeat(0).take(speeds_size)),
        })
    })?;
    Ok(())
}

fn read_directed_edge_count(extract: &mut File, tile: &ArchivedTile) -> io::Result<u32> {
    let mut record_counts = [0; 8];
    if u64::from(tile.entry.size) < RECORD_COUNTS_OFFSET + record_counts.len() as u64 {
        return Err(io::ErrorKind::UnexpectedEof.into());
    }
    extract.seek(SeekFrom::Start(tile.entry.offset + RECORD_COUNTS_OFFSET))?;
    extract.read_exact(&mut record_counts)?;
    Ok(directed_edge_count(record_counts))
}

/// The bit field holds 21 bits of node count, then 21 bits of directed edge count.
fn directed_edge_count(record_counts: [u8; 8]) -> u32 {
    const COUNT_MASK: u64 = (1 << 21) - 1;
    let directed_edges = (u64::from_le_bytes(record_counts) >> 21) & COUNT_MASK;
    // Lossless: the mask keeps 21 bits.
    directed_edges as u32
}

/// `TrafficTileHeader` with no update time yet.
fn traffic_header(tile_id: u32, directed_edge_count: u32) -> Vec<u8> {
    let mut header = Vec::with_capacity(32);
    header.extend_from_slice(&u64::from(tile_id).to_le_bytes());
    header.extend_from_slice(&0_u64.to_le_bytes()); // last_update
    header.extend_from_slice(&directed_edge_count.to_le_bytes());
    header.extend_from_slice(&TRAFFIC_TILE_VERSION.to_le_bytes());
    header.extend_from_slice(&0_u32.to_le_bytes()); // spare2
    header.extend_from_slice(&0_u32.to_le_bytes()); // spare3
    header
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record_counts(nodes: u64, directed_edges: u64, predicted_speeds: u64) -> [u8; 8] {
        (nodes | directed_edges << 21 | predicted_speeds << 42).to_le_bytes()
    }

    #[test]
    fn the_directed_edge_count_is_read_between_the_node_and_speed_counts() {
        let counts = record_counts(0x1f_ffff, 123_456, 0x1f_ffff);
        assert_eq!(directed_edge_count(counts), 123_456);
    }

    #[test]
    fn the_largest_directed_edge_count_fits_in_21_bits() {
        assert_eq!(
            directed_edge_count(record_counts(0, 0x1f_ffff, 0)),
            0x1f_ffff
        );
    }

    #[test]
    fn the_traffic_header_holds_tile_id_update_time_edge_count_version_and_spares() {
        // Tile 2/000/818/660 (id 0x63ef22) with 1000 (0x3e8) directed edges.
        #[rustfmt::skip]
        let expected = [
            0x22, 0xef, 0x63, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0,
            0xe8, 0x03, 0, 0,
            3, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
        ];
        assert_eq!(traffic_header(6_549_282, 1000), expected);
    }
}
