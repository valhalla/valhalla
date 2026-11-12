use std::fs;
use std::io::{self, Read, Write};
use std::net::TcpListener;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use serde_json::{Value, json};
use tempfile::TempDir;

struct FakeTile {
    name: &'static str,
    tile_id: u32,
    directed_edges: u64,
    size: usize,
}

const UTRECHT: [FakeTile; 3] = [
    FakeTile {
        name: "0/003/196.gph",
        tile_id: 25568,
        directed_edges: 10,
        size: 1000,
    },
    FakeTile {
        name: "1/051/305.gph",
        tile_id: 410441,
        directed_edges: 3,
        size: 512,
    },
    FakeTile {
        name: "2/000/818/660.gph",
        tile_id: 6549282,
        directed_edges: 700,
        size: 3001,
    },
];

/// The level 2 tile in the south-west corner of the world.
const ANTARCTIC: FakeTile = FakeTile {
    name: "2/000/000/000.gph",
    tile_id: 2,
    directed_edges: 1,
    size: 48,
};

/// Bytes shaped like a graph tile: the record counts at byte 40, then an arbitrary payload.
fn tile_bytes(tile: &FakeTile) -> Vec<u8> {
    let mut bytes = vec![0xab; 40];
    let record_counts = 0x1f_ffff | (tile.directed_edges << 21) | (5 << 42);
    bytes.extend_from_slice(&u64::to_le_bytes(record_counts));
    bytes.extend((48..tile.size).map(|i| (i % 251) as u8));
    bytes
}

fn build_extract(config: &Value) -> Command {
    let mut command = Command::new(env!("CARGO_BIN_EXE_valhalla_build_extract"));
    command.arg("--inline-config").arg(config.to_string());
    command
}

struct Workspace {
    dir: TempDir,
}

impl Workspace {
    fn with_tiles(tiles: &[&FakeTile]) -> Self {
        let workspace = Self {
            dir: tempfile::tempdir().unwrap(),
        };
        for tile in tiles {
            let path = workspace.tile_dir().join(tile.name);
            fs::create_dir_all(path.parent().unwrap()).unwrap();
            fs::write(path, tile_bytes(tile)).unwrap();
        }
        workspace
    }

    fn utrecht() -> Self {
        Self::with_tiles(&UTRECHT.iter().collect::<Vec<_>>())
    }

    fn tile_dir(&self) -> PathBuf {
        self.dir.path().join("tiles")
    }

    fn path(&self, name: &str) -> PathBuf {
        self.dir.path().join(name)
    }

    fn config(&self) -> Value {
        json!({"mjolnir": {
            "tile_dir": self.tile_dir(),
            "tile_extract": self.path("tiles.tar"),
        }})
    }

    fn build_extract(&self, args: &[&str]) -> Output {
        build_extract(&self.config()).args(args).output().unwrap()
    }
}

fn assert_success(output: &Output) {
    assert!(
        output.status.success(),
        "exit {:?}: {}",
        output.status.code(),
        String::from_utf8_lossy(&output.stderr)
    );
}

const BLOCK_SIZE: usize = 512;

#[derive(Debug)]
struct Member {
    name: String,
    data_offset: usize,
    data: Vec<u8>,
}

/// Walks the tar block by block like `midgard::tar` in valhalla/midgard/sequence.h, failing
/// on anything that reader would skip or misread.
fn read_tar(path: &Path) -> Vec<Member> {
    let bytes = fs::read(path).unwrap();
    assert_eq!(bytes.len() % BLOCK_SIZE, 0, "not a whole number of blocks");
    let mut members = Vec::new();
    let mut position = 0;
    loop {
        let header = &bytes[position..position + BLOCK_SIZE];
        if header.iter().all(|&byte| byte == 0) {
            assert_eq!(
                &bytes[position..],
                &[0; 2 * BLOCK_SIZE][..],
                "the tar must end with exactly two zero blocks"
            );
            return members;
        }
        assert_eq!(
            octal(&header[148..156]),
            checksum(header),
            "checksum at {position}"
        );
        assert_eq!(header[156], b'0', "typeflag at {position}");
        assert_eq!(&header[257..263], b"ustar\0", "magic at {position}");
        assert!(
            header[345..500].iter().all(|&byte| byte == 0),
            "the reader ignores the ustar prefix field, so names must fit in 100 bytes"
        );
        let name_length = header[..100]
            .iter()
            .position(|&byte| byte == 0)
            .unwrap_or(100);
        let size = usize::try_from(octal(&header[124..136])).unwrap();
        let data_offset = position + BLOCK_SIZE;
        members.push(Member {
            name: String::from_utf8(header[..name_length].to_vec()).unwrap(),
            data_offset,
            data: bytes[data_offset..data_offset + size].to_vec(),
        });
        position = data_offset + size.next_multiple_of(BLOCK_SIZE);
    }
}

/// Sum of the header bytes with the checksum field counted as spaces.
fn checksum(header: &[u8]) -> u64 {
    let field = 148..156;
    header
        .iter()
        .enumerate()
        .map(|(i, &byte)| u64::from(if field.contains(&i) { b' ' } else { byte }))
        .sum()
}

/// Octal digits up to the first NUL or space, as `octal_to_int` reads them.
fn octal(field: &[u8]) -> u64 {
    let digits: Vec<u8> = field
        .iter()
        .copied()
        .take_while(|&byte| byte != 0 && byte != b' ')
        .collect();
    u64::from_str_radix(std::str::from_utf8(&digits).unwrap(), 8).unwrap()
}

/// `(offset, tile_id, size)` triples of the `index.bin` member.
fn read_index(path: &Path) -> Vec<(u64, u32, u32)> {
    let members = read_tar(path);
    let (entries, rest) = members[0].data.as_chunks::<16>();
    assert!(rest.is_empty(), "index.bin size is not a multiple of 16");
    entries
        .iter()
        .map(|entry| {
            (
                u64::from_le_bytes(entry[..8].try_into().unwrap()),
                u32::from_le_bytes(entry[8..12].try_into().unwrap()),
                u32::from_le_bytes(entry[12..].try_into().unwrap()),
            )
        })
        .collect()
}

fn index_of_members(path: &Path, tiles: &[&FakeTile]) -> Vec<(u64, u32, u32)> {
    read_tar(path)
        .iter()
        .skip(1)
        .zip(tiles)
        .map(|(member, tile)| {
            (
                member.data_offset.try_into().unwrap(),
                tile.tile_id,
                member.data.len().try_into().unwrap(),
            )
        })
        .collect()
}

fn member_names(path: &Path) -> Vec<String> {
    read_tar(path)
        .into_iter()
        .map(|member| member.name)
        .collect()
}

#[test]
fn index_bin_is_the_first_member_and_its_data_starts_at_byte_512() {
    let workspace = Workspace::utrecht();
    assert_success(&workspace.build_extract(&[]));

    let first = &read_tar(&workspace.path("tiles.tar"))[0];
    assert_eq!((first.name.as_str(), first.data_offset), ("index.bin", 512));
}

#[test]
fn tiles_are_stored_under_their_relative_path_in_sorted_order() {
    let workspace = Workspace::utrecht();
    assert_success(&workspace.build_extract(&[]));

    let expected = [
        "index.bin",
        UTRECHT[0].name,
        UTRECHT[1].name,
        UTRECHT[2].name,
    ];
    assert_eq!(member_names(&workspace.path("tiles.tar")), expected);
}

#[test]
fn an_index_entry_points_at_the_tile_data() {
    let workspace = Workspace::utrecht();
    assert_success(&workspace.build_extract(&[]));

    let extract = workspace.path("tiles.tar");
    let tiles: Vec<_> = UTRECHT.iter().collect();
    assert_eq!(read_index(&extract), index_of_members(&extract, &tiles));
}

#[test]
fn the_bytes_at_each_index_offset_are_the_source_tile() {
    let workspace = Workspace::utrecht();
    assert_success(&workspace.build_extract(&[]));

    let extract = fs::read(workspace.path("tiles.tar")).unwrap();
    let stored: Vec<_> = read_index(&workspace.path("tiles.tar"))
        .into_iter()
        .map(|(offset, _, size)| extract[offset as usize..][..size as usize].to_vec())
        .collect();
    let sources: Vec<_> = UTRECHT.iter().map(tile_bytes).collect();
    assert_eq!(stored, sources);
}

#[test]
fn the_extract_is_a_whole_number_of_tar_blocks() {
    let workspace = Workspace::utrecht();
    assert_success(&workspace.build_extract(&[]));

    let size = fs::metadata(workspace.path("tiles.tar")).unwrap().len();
    assert_eq!(size % 512, 0);
}

#[test]
fn each_traffic_tile_holds_a_header_and_one_zeroed_speed_per_directed_edge() {
    let workspace = Workspace::utrecht();
    assert_success(&workspace.build_extract(&["--with-traffic"]));

    let traffic_tiles: Vec<_> = read_tar(&workspace.path("traffic.tar"))
        .into_iter()
        .skip(1)
        .map(|member| (member.name, member.data))
        .collect();
    let expected: Vec<_> = UTRECHT
        .iter()
        .map(|tile| {
            let mut data = Vec::new();
            data.extend_from_slice(&u64::from(tile.tile_id).to_le_bytes());
            data.extend_from_slice(&0_u64.to_le_bytes());
            data.extend_from_slice(&(tile.directed_edges as u32).to_le_bytes());
            data.extend_from_slice(&3_u32.to_le_bytes());
            data.extend_from_slice(&[0; 8]);
            data.resize(32 + 8 * tile.directed_edges as usize, 0);
            (tile.name.to_owned(), data)
        })
        .collect();
    assert_eq!(traffic_tiles, expected);
}

#[test]
fn the_traffic_index_points_at_the_traffic_tiles() {
    let workspace = Workspace::utrecht();
    assert_success(&workspace.build_extract(&["-t"]));

    let traffic = workspace.path("traffic.tar");
    let tiles: Vec<_> = UTRECHT.iter().collect();
    assert_eq!(read_index(&traffic), index_of_members(&traffic, &tiles));
}

#[test]
fn the_traffic_extract_is_a_whole_number_of_tar_blocks() {
    let workspace = Workspace::utrecht();
    assert_success(&workspace.build_extract(&["-t"]));

    let size = fs::metadata(workspace.path("traffic.tar")).unwrap().len();
    assert_eq!(size % 512, 0);
}

#[test]
fn the_traffic_extract_goes_to_mjolnir_traffic_extract_when_set() {
    let workspace = Workspace::utrecht();
    let mut config = workspace.config();
    config["mjolnir"]["traffic_extract"] = json!(workspace.path("live/speeds.tar"));

    assert_success(&build_extract(&config).arg("-t").output().unwrap());

    let written = (
        workspace.path("live/speeds.tar").is_file(),
        workspace.path("traffic.tar").exists(),
    );
    assert_eq!(written, (true, false));
}

#[test]
fn an_existing_extract_is_left_untouched_without_overwrite() {
    let workspace = Workspace::utrecht();
    fs::write(workspace.path("tiles.tar"), "keep me").unwrap();

    let output = workspace.build_extract(&[]);

    assert_eq!(output.status.code(), Some(1));
    assert_eq!(fs::read(workspace.path("tiles.tar")).unwrap(), b"keep me");
}

#[test]
fn overwrite_replaces_an_existing_extract() {
    let workspace = Workspace::utrecht();
    fs::write(workspace.path("tiles.tar"), "replace me").unwrap();

    assert_success(&workspace.build_extract(&["--overwrite"]));

    assert_eq!(member_names(&workspace.path("tiles.tar")).len(), 4);
}

#[test]
fn a_bbox_keeps_only_the_tiles_it_touches() {
    let workspace = Workspace::with_tiles(&[&UTRECHT[0], &UTRECHT[2], &ANTARCTIC]);

    assert_success(&workspace.build_extract(&["--bbox", "5.0,52.0,5.1,52.1"]));

    let expected = ["index.bin", UTRECHT[0].name, UTRECHT[2].name];
    assert_eq!(member_names(&workspace.path("tiles.tar")), expected);
}

#[test]
fn a_bbox_with_negative_values_is_accepted_as_a_separate_argument() {
    let workspace = Workspace::with_tiles(&[&UTRECHT[0], &ANTARCTIC]);

    assert_success(&workspace.build_extract(&["-b", "-180,-90,-179.9,-89.9"]));

    assert_eq!(
        member_names(&workspace.path("tiles.tar")),
        ["index.bin", ANTARCTIC.name]
    );
}

#[test]
fn a_bbox_touching_no_tile_is_an_error() {
    let workspace = Workspace::utrecht();

    let output = workspace.build_extract(&["--bbox", "100,10,101,11"]);

    assert_eq!(output.status.code(), Some(1));
}

#[test]
fn an_invalid_bbox_is_a_usage_error() {
    let workspace = Workspace::utrecht();

    let output = workspace.build_extract(&["--bbox", "5,52,4,53"]);

    assert_eq!(output.status.code(), Some(2));
}

#[test]
fn a_bbox_and_a_region_together_are_a_usage_error() {
    let workspace = Workspace::utrecht();

    let output = workspace.build_extract(&["--bbox", "5,52,6,53", "--region", "europe"]);

    assert_eq!(output.status.code(), Some(2));
}

fn write_geojson(dir: &Path, name: &str, geometries: &[Value]) {
    let features: Vec<_> = geometries
        .iter()
        .map(|geometry| json!({"type": "Feature", "properties": {}, "geometry": geometry}))
        .collect();
    let collection = json!({"type": "FeatureCollection", "features": features});
    fs::create_dir_all(dir).unwrap();
    fs::write(dir.join(name), collection.to_string()).unwrap();
}

fn triangle(x: f64, y: f64, size: f64) -> Value {
    json!({"type": "Polygon", "coordinates": [[[x, y], [x + size, y], [x, y + size], [x, y]]]})
}

#[test]
fn a_geojson_dir_keeps_the_tiles_its_polygons_touch() {
    let workspace = Workspace::with_tiles(&[&UTRECHT[0], &UTRECHT[1], &ANTARCTIC]);
    let geojson_dir = workspace.path("geojson");
    write_geojson(&geojson_dir, "utrecht.geojson", &[triangle(5.0, 52.0, 0.2)]);

    assert_success(&workspace.build_extract(&["--geojson-dir", geojson_dir.to_str().unwrap()]));

    let expected = ["index.bin", UTRECHT[0].name, UTRECHT[1].name];
    assert_eq!(member_names(&workspace.path("tiles.tar")), expected);
}

#[test]
fn every_geojson_file_of_the_dir_is_used() {
    let elsewhere = FakeTile {
        name: "2/000/000/010.gph",
        tile_id: 82,
        directed_edges: 1,
        size: 48,
    };
    let workspace = Workspace::with_tiles(&[&UTRECHT[2], &ANTARCTIC, &elsewhere]);
    let geojson_dir = workspace.path("geojson");
    write_geojson(
        &geojson_dir,
        "utrecht.geojson",
        &[triangle(5.1, 52.1, 0.01)],
    );
    write_geojson(
        &geojson_dir,
        "pole.geojson",
        &[triangle(-179.9, -89.9, 0.01)],
    );

    assert_success(&workspace.build_extract(&["--geojson-dir", geojson_dir.to_str().unwrap()]));

    let expected = ["index.bin", ANTARCTIC.name, UTRECHT[2].name];
    assert_eq!(member_names(&workspace.path("tiles.tar")), expected);
}

#[test]
fn a_geojson_feature_without_geometry_is_skipped() {
    let workspace = Workspace::with_tiles(&[&UTRECHT[2], &ANTARCTIC]);
    let geojson_dir = workspace.path("geojson");
    write_geojson(
        &geojson_dir,
        "utrecht.geojson",
        &[Value::Null, triangle(5.1, 52.1, 0.01)],
    );

    assert_success(&workspace.build_extract(&["--geojson-dir", geojson_dir.to_str().unwrap()]));

    assert_eq!(
        member_names(&workspace.path("tiles.tar")),
        ["index.bin", UTRECHT[2].name]
    );
}

#[cfg(unix)]
#[test]
fn a_symlinked_tile_is_stored_as_a_regular_member_with_its_target_content() {
    let workspace = Workspace::with_tiles(&[&UTRECHT[0]]);
    let target = workspace.path("elsewhere/660.gph");
    fs::create_dir_all(target.parent().unwrap()).unwrap();
    fs::write(&target, tile_bytes(&UTRECHT[2])).unwrap();
    let link = workspace.tile_dir().join(UTRECHT[2].name);
    fs::create_dir_all(link.parent().unwrap()).unwrap();
    std::os::unix::fs::symlink(&target, &link).unwrap();

    assert_success(&workspace.build_extract(&[]));

    let stored = read_tar(&workspace.path("tiles.tar")).pop().unwrap();
    assert_eq!(
        (stored.name.as_str(), stored.data),
        (UTRECHT[2].name, tile_bytes(&UTRECHT[2]))
    );
}

#[test]
fn an_extract_rebuilt_from_a_tar_has_the_same_index() {
    let workspace = Workspace::utrecht();
    assert_success(&workspace.build_extract(&[]));
    let copy = workspace.path("copy.tar");

    assert_success(&workspace.build_extract(&["--extract-tar", copy.to_str().unwrap()]));

    assert_eq!(read_index(&copy), read_index(&workspace.path("tiles.tar")));
}

#[test]
fn an_extract_can_be_rebuilt_in_place_from_itself() {
    let workspace = Workspace::with_tiles(&[&UTRECHT[0], &UTRECHT[2], &ANTARCTIC]);
    assert_success(&workspace.build_extract(&[]));
    fs::remove_dir_all(workspace.tile_dir()).unwrap();
    let extract = workspace.path("tiles.tar");

    let args = ["-e", extract.to_str().unwrap(), "-O", "-b", "5,52,5.1,52.1"];
    assert_success(&workspace.build_extract(&args));

    let expected = ["index.bin", UTRECHT[0].name, UTRECHT[2].name];
    assert_eq!(member_names(&extract), expected);
}

#[test]
fn extract_tar_without_mjolnir_tile_extract_reads_the_tile_dir() {
    let workspace = Workspace::utrecht();
    let config = json!({"mjolnir": {"tile_dir": workspace.tile_dir()}});
    let output = workspace.path("out.tar");

    let run = build_extract(&config)
        .args(["-e", output.to_str().unwrap()])
        .output()
        .unwrap();
    assert_success(&run);

    assert_eq!(member_names(&output).len(), 1 + UTRECHT.len());
}

#[test]
fn a_failed_traffic_extract_leaves_no_file_behind() {
    let workspace = Workspace::with_tiles(&[&UTRECHT[0]]);
    // Too short to hold the record counts the traffic tile is sized from.
    fs::write(workspace.tile_dir().join("0/003/197.gph"), [0; 20]).unwrap();

    let output = workspace.build_extract(&["--with-traffic"]);

    assert_eq!(output.status.code(), Some(1));
    let mut entries: Vec<_> = fs::read_dir(workspace.dir.path())
        .unwrap()
        .map(|entry| entry.unwrap().file_name().into_string().unwrap())
        .collect();
    entries.sort();
    assert_eq!(entries, ["tiles", "tiles.tar"]);
}

#[test]
fn a_config_without_mjolnir_is_an_error() {
    let output = build_extract(&json!({})).output().unwrap();

    assert_eq!(output.status.code(), Some(1));
}

#[test]
fn an_empty_tile_dir_is_reported_as_such() {
    let workspace = Workspace::with_tiles(&[]);
    fs::create_dir(workspace.tile_dir()).unwrap();

    let output = workspace.build_extract(&[]);

    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(stderr.contains("no tiles found in"), "stderr: {stderr}");
}

#[test]
fn errors_are_reported_on_stderr() {
    let workspace = Workspace::utrecht();
    fs::remove_dir_all(workspace.tile_dir()).unwrap();

    let output = workspace.build_extract(&[]);

    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(stderr.contains("mjolnir.tile_dir"), "stderr: {stderr}");
}

/// A local HTTP server that answers exactly one request.
struct FakeServer {
    url: String,
    thread: JoinHandle<()>,
}

impl FakeServer {
    /// Reads one request's headers, writes `response` verbatim and closes the connection.
    fn answer_once(response: String) -> Self {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}/index-v1.json", listener.local_addr().unwrap());
        // Non-blocking accept, so a client that never connects fails the test instead of hanging it.
        listener.set_nonblocking(true).unwrap();
        let thread = thread::spawn(move || {
            let deadline = Instant::now() + Duration::from_secs(10);
            let mut stream = loop {
                match listener.accept() {
                    Ok((stream, _)) => break stream,
                    Err(error)
                        if error.kind() == io::ErrorKind::WouldBlock
                            && Instant::now() < deadline =>
                    {
                        thread::sleep(Duration::from_millis(5));
                    }
                    Err(error) => panic!("no request received: {error}"),
                }
            };
            stream.set_nonblocking(false).unwrap();
            let mut request = Vec::new();
            let mut buffer = [0; 1024];
            while !request.ends_with(b"\r\n\r\n") {
                let read = stream.read(&mut buffer).unwrap();
                assert_ne!(read, 0, "connection closed before the end of the request");
                request.extend_from_slice(&buffer[..read]);
            }
            assert!(request.starts_with(b"GET /index-v1.json "));
            stream.write_all(response.as_bytes()).unwrap();
        });
        Self { url, thread }
    }

    /// Fails the calling test if the server thread failed.
    fn join(self) {
        if let Err(panic) = self.thread.join() {
            std::panic::resume_unwind(panic);
        }
    }
}

fn http_ok(body: &str) -> String {
    format!(
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}",
        body.len()
    )
}

fn geofabrik_index() -> String {
    json!({"type": "FeatureCollection", "features": [
        {
            "type": "Feature",
            "properties": {"id": "europe", "name": "Europe", "parent": ""},
            "geometry": {"type": "Polygon", "coordinates": [[[-25, 35], [45, 35], [45, 72], [-25, 72], [-25, 35]]]}
        },
        {
            "type": "Feature",
            "properties": {"id": "utrecht", "name": "Utrecht", "parent": "europe"},
            "geometry": {"type": "MultiPolygon", "coordinates": [[[[5.0, 52.0], [5.2, 52.0], [5.2, 52.2], [5.0, 52.0]]]]}
        }
    ]})
    .to_string()
}

fn build_extract_with_regions(workspace: &Workspace, server: &FakeServer, region: &str) -> Output {
    build_extract(&workspace.config())
        .env("VALHALLA_GEOFABRIK_INDEX_URL", &server.url)
        .args(["--region", region])
        .output()
        .unwrap()
}

#[test]
fn a_region_keeps_the_tiles_its_boundary_touches() {
    let workspace = Workspace::with_tiles(&[&UTRECHT[1], &UTRECHT[2], &ANTARCTIC]);
    let server = FakeServer::answer_once(http_ok(&geofabrik_index()));

    let output = build_extract_with_regions(&workspace, &server, "europe/utrecht");
    server.join();
    assert_success(&output);

    let expected = ["index.bin", UTRECHT[1].name, UTRECHT[2].name];
    assert_eq!(member_names(&workspace.path("tiles.tar")), expected);
}

#[test]
fn region_list_prints_the_regions_on_stdout_without_a_config() {
    let server = FakeServer::answer_once(http_ok(&geofabrik_index()));

    let output = Command::new(env!("CARGO_BIN_EXE_valhalla_build_extract"))
        .env("VALHALLA_GEOFABRIK_INDEX_URL", &server.url)
        .args(["--region", "list"])
        .output()
        .unwrap();
    server.join();
    assert_success(&output);

    let expected = format!(
        "\n(top-level):\n  {:40} europe\n\neurope:\n  {:40} utrecht\n",
        "Europe", "Utrecht"
    );
    assert_eq!(String::from_utf8_lossy(&output.stdout), expected);
}

#[test]
fn a_geofabrik_index_answering_an_http_error_is_reported_with_its_status() {
    let workspace = Workspace::utrecht();
    let not_found = "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
    let server = FakeServer::answer_once(not_found.to_owned());

    let output = build_extract_with_regions(&workspace, &server, "utrecht");
    server.join();

    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        output.status.code() == Some(1) && stderr.contains("404"),
        "exit {:?}, stderr: {stderr}",
        output.status.code()
    );
}

#[test]
fn a_geofabrik_index_closing_the_connection_without_answer_is_an_error() {
    let workspace = Workspace::utrecht();
    let server = FakeServer::answer_once(String::new());

    let output = build_extract_with_regions(&workspace, &server, "utrecht");
    server.join();

    assert_eq!(output.status.code(), Some(1));
}

fn is_number_of(text: &str, digits: usize) -> bool {
    text.len() == digits && text.bytes().all(|byte| byte.is_ascii_digit())
}

/// Checks a leading `YYYY-MM-DD HH:MM:SS` and returns what follows it.
fn after_date_time(line: &str) -> &str {
    let (date, rest) = line.split_once(' ').unwrap();
    let date: Vec<_> = date.split('-').collect();
    let time: Vec<_> = rest[..8].split(':').collect();
    let well_formed = date.len() == 3
        && is_number_of(date[0], 4)
        && is_number_of(date[1], 2)
        && is_number_of(date[2], 2)
        && time.len() == 3
        && time.iter().all(|part| is_number_of(part, 2));
    assert!(well_formed, "no date and time at the start of {line:?}");
    &rest[8..]
}

fn extract_log_line(output: &Output) -> String {
    let stderr = String::from_utf8_lossy(&output.stderr);
    stderr
        .lines()
        .find(|line| line.contains("wrote the tile extract"))
        .unwrap_or_else(|| panic!("no log line in {stderr:?}"))
        .to_owned()
}

/// `2026-11-12 12:59:04,737  INFO: message key=value`
fn assert_python_log_line(line: &str) {
    let rest = after_date_time(line);
    let (millis, rest) = rest.strip_prefix(',').unwrap().split_at(3);
    assert!(is_number_of(millis, 3), "milliseconds in {line:?}");
    assert!(
        rest.starts_with("  INFO: wrote the tile extract tiles=3 path="),
        "level and message in {line:?}"
    );
}

#[test]
fn log_lines_use_the_python_format_by_default() {
    let workspace = Workspace::utrecht();

    let output = workspace.build_extract(&["-v"]);

    assert_python_log_line(&extract_log_line(&output));
}

#[test]
fn the_python_log_format_has_milliseconds_and_a_right_aligned_level() {
    let workspace = Workspace::utrecht();

    let output = workspace.build_extract(&["-v", "--log-format", "python"]);

    assert_python_log_line(&extract_log_line(&output));
}

#[test]
fn the_valhalla_log_format_has_nanoseconds_and_a_bracketed_level() {
    let workspace = Workspace::utrecht();

    let output = workspace.build_extract(&["-v", "--log-format", "valhalla"]);

    let line = extract_log_line(&output);
    let rest = after_date_time(&line);
    let (nanos, rest) = rest.strip_prefix('.').unwrap().split_at(9);
    assert!(is_number_of(nanos, 9), "nanoseconds in {line:?}");
    assert!(
        rest.starts_with(" [INFO] wrote the tile extract tiles=3 path="),
        "level and message in {line:?}"
    );
}

#[test]
fn in_place_archives_pass_the_format_checks_over_a_longer_old_file() {
    let workspace = Workspace::utrecht();
    for name in ["tiles.tar", "traffic.tar"] {
        fs::write(workspace.path(name), vec![0xff; 100_000]).unwrap();
    }

    assert_success(&workspace.build_extract(&["--in-place", "-O", "-t"]));

    let tiles: Vec<_> = UTRECHT.iter().collect();
    let indexes_match = ["tiles.tar", "traffic.tar"].map(|name| {
        let path = workspace.path(name);
        read_index(&path) == index_of_members(&path, &tiles)
    });
    assert_eq!(indexes_match, [true, true]);
}

#[test]
fn in_place_refuses_to_rewrite_its_input_tar() {
    let workspace = Workspace::utrecht();
    assert_success(&workspace.build_extract(&[]));
    let extract = workspace.path("tiles.tar");
    let before = fs::read(&extract).unwrap();

    let output = workspace.build_extract(&["--in-place", "-O", "-e", extract.to_str().unwrap()]);

    assert_eq!(
        (output.status.code(), fs::read(&extract).unwrap()),
        (Some(1), before)
    );
}

#[cfg(unix)]
#[test]
fn in_place_recognises_its_input_tar_behind_a_hard_link() {
    let workspace = Workspace::utrecht();
    assert_success(&workspace.build_extract(&[]));
    let extract = workspace.path("tiles.tar");
    let alias = workspace.path("alias.tar");
    fs::hard_link(&extract, &alias).unwrap();
    let before = fs::read(&extract).unwrap();

    let output = workspace.build_extract(&["--in-place", "-O", "-e", alias.to_str().unwrap()]);

    assert_eq!(
        (output.status.code(), fs::read(&extract).unwrap()),
        (Some(1), before)
    );
}

#[cfg(unix)]
fn symlinked_output(workspace: &Workspace) -> PathBuf {
    let target = workspace.path("real/tiles.tar");
    fs::create_dir_all(target.parent().unwrap()).unwrap();
    fs::write(&target, "old").unwrap();
    std::os::unix::fs::symlink(&target, workspace.path("tiles.tar")).unwrap();
    target
}

#[cfg(unix)]
#[test]
fn in_place_writes_through_a_symlinked_output() {
    let workspace = Workspace::utrecht();
    let target = symlinked_output(&workspace);

    assert_success(&workspace.build_extract(&["--in-place", "-O"]));

    let link = fs::symlink_metadata(workspace.path("tiles.tar")).unwrap();
    assert_eq!(
        (link.is_symlink(), member_names(&target).len()),
        (true, 1 + UTRECHT.len())
    );
}

#[cfg(unix)]
#[test]
fn without_in_place_a_symlinked_output_is_replaced_by_a_regular_file() {
    let workspace = Workspace::utrecht();
    let target = symlinked_output(&workspace);

    assert_success(&workspace.build_extract(&["-O"]));

    let link = fs::symlink_metadata(workspace.path("tiles.tar")).unwrap();
    assert_eq!(
        (link.is_file(), fs::read(&target).unwrap()),
        (true, b"old".to_vec())
    );
}

#[cfg(unix)]
#[test]
fn in_place_keeps_the_permissions_of_an_existing_output() {
    use std::os::unix::fs::PermissionsExt;

    let workspace = Workspace::utrecht();
    let extract = workspace.path("tiles.tar");
    fs::write(&extract, "old").unwrap();
    fs::set_permissions(&extract, fs::Permissions::from_mode(0o640)).unwrap();

    assert_success(&workspace.build_extract(&["--in-place", "-O"]));

    let mode = fs::metadata(&extract).unwrap().permissions().mode();
    assert_eq!(mode & 0o777, 0o640);
}
