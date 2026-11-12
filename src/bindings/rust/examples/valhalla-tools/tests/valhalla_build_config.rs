use std::fs;
use std::path::Path;
use std::process::{Command, Output};

use tempfile::TempDir;
use valhalla_tools::build_config::{
    self, Arguments, DefaultValue, LEAVES, MergeReport, OptionalKind, Parsed, UsageError, Value,
    merge_json, should_remove_key,
};

fn run(dir: &Path, args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_valhalla_build_config"))
        .current_dir(dir)
        .args(args)
        .output()
        .unwrap()
}

fn stdout(output: &Output) -> &str {
    std::str::from_utf8(&output.stdout).unwrap()
}

fn stderr(output: &Output) -> &str {
    std::str::from_utf8(&output.stderr).unwrap()
}

/// The generated config, read back.
fn generated(args: &[&str]) -> Value {
    let dir = TempDir::new().unwrap();
    let output = run(dir.path(), args);
    assert_eq!(output.status.code(), Some(0), "{}", stderr(&output));
    build_config::parse(stdout(&output)).unwrap()
}

fn json(text: &str) -> Value {
    build_config::parse(text).unwrap()
}

fn get<'a>(value: &'a Value, path: &str) -> Option<&'a Value> {
    path.split('.').try_fold(value, |node, key| match node {
        Value::Dict(entries) => entries.get(key),
        _ => None,
    })
}

fn keys(value: &Value) -> Vec<&str> {
    match value {
        Value::Dict(entries) => entries.keys().map(String::as_str).collect(),
        _ => Vec::new(),
    }
}

fn parsed(args: &[&str]) -> Result<Parsed, UsageError> {
    let args: Vec<String> = args.iter().map(|arg| (*arg).to_owned()).collect();
    build_config::parse_args(&args)
}

fn arguments(args: &[&str]) -> Arguments {
    match parsed(args) {
        Ok(Parsed::Run(arguments)) => arguments,
        other => panic!("{other:?}"),
    }
}

fn leaf_value(args: &[&str], option: &str) -> Option<Value> {
    let index = LEAVES
        .iter()
        .position(|leaf| leaf.option == option)
        .unwrap();
    arguments(args).values[index].clone()
}

fn int(digits: &str) -> Value {
    Value::Int(digits.to_owned())
}

fn string(text: &str) -> Value {
    Value::Str(text.to_owned())
}

fn merged(old: &str, new: &str, remove_keys: Option<&[&str]>) -> (Value, MergeReport) {
    let remove_keys: Option<Vec<String>> =
        remove_keys.map(|keys| keys.iter().map(|key| (*key).to_owned()).collect());
    let mut report = MergeReport::default();
    let result = merge_json(
        &json(old),
        &json(new),
        &mut report,
        remove_keys.as_deref(),
        "",
    );
    (result, report)
}

// Ported from test/scripts/test_valhalla_build_config.py: leaves and their types.

#[test]
fn leaves_parse_values_with_their_type() {
    assert_eq!(
        leaf_value(&["--mjolnir-tile-dir", "/tiles"], "mjolnir-tile-dir"),
        Some(string("/tiles"))
    );
    assert_eq!(
        leaf_value(
            &["--mjolnir-max-cache-size", "100"],
            "mjolnir-max-cache-size"
        ),
        Some(int("100"))
    );
    assert_eq!(
        leaf_value(
            &["--meili-default-sigma-z", "10.9"],
            "meili-default-sigma-z"
        ),
        Some(Value::Float(10.9))
    );
    assert_eq!(
        leaf_value(&["--mjolnir-user-agent", "agent"], "mjolnir-user-agent"),
        Some(string("agent"))
    );
    assert_eq!(
        leaf_value(&["--mjolnir-concurrency", "4"], "mjolnir-concurrency"),
        Some(int("4"))
    );
}

#[test]
fn optional_list_leaves_take_one_or_more_arguments() {
    assert_eq!(
        leaf_value(&["--statsd-tags", "a,b", "c"], "statsd-tags"),
        Some(Value::List(vec![string("a,b"), string("c")]))
    );
    assert_eq!(
        parsed(&["--statsd-tags"]),
        Err(UsageError::ExpectedAtLeastOne("--statsd-tags".into()))
    );
}

#[test]
fn list_leaves_split_one_argument_on_commas() {
    assert_eq!(
        leaf_value(&["--loki-actions", "route,"], "loki-actions"),
        Some(Value::List(vec![string("route"), string("")]))
    );
}

#[test]
fn bool_leaves_accept_strtobool_spellings() {
    assert_eq!(
        leaf_value(&["--logging-color", "OFF"], "logging-color"),
        Some(Value::Bool(false))
    );
    assert!(matches!(
        parsed(&["--logging-color", "maybe"]),
        Err(UsageError::InvalidValue { .. })
    ));
}

#[test]
fn optional_bool_leaves_take_strtobool_spellings() {
    assert_eq!(
        leaf_value(&["--mjolnir-tile-url-gz", "false"], "mjolnir-tile-url-gz"),
        Some(Value::Bool(false))
    );
    assert_eq!(
        leaf_value(&["--mjolnir-tile-url-gz", "Yes"], "mjolnir-tile-url-gz"),
        Some(Value::Bool(true))
    );
}

#[test]
fn optional_bool_leaves_refuse_other_values() {
    assert!(matches!(
        parsed(&["--mjolnir-tile-url-gz", ""]),
        Err(UsageError::InvalidValue { .. })
    ));
}

#[test]
fn leaves_are_unique_and_in_dict_order() {
    let paths: Vec<String> = LEAVES.iter().map(|leaf| leaf.path.join(".")).collect();
    let mut unique = paths.clone();
    unique.sort();
    unique.dedup();
    assert_eq!(unique.len(), paths.len());
    assert_eq!(
        paths[..3],
        ["logging.type", "logging.color", "logging.file_name"]
    );
    assert_eq!(paths.last().map(String::as_str), Some("statsd.tags"));
}

#[test]
fn optional_leaves_are_left_out_unless_given() {
    let config = build_config::config(&vec![None; LEAVES.len()]);
    assert_eq!(get(&config, "statsd.host"), None);
    let index = LEAVES
        .iter()
        .position(|leaf| leaf.option == "statsd-host")
        .unwrap();
    let mut values = vec![None; LEAVES.len()];
    values[index] = Some(string("localhost"));
    assert_eq!(
        get(&build_config::config(&values), "statsd.host"),
        Some(&string("localhost"))
    );
}

#[test]
fn every_optional_kind_is_known() {
    for leaf in LEAVES {
        if let DefaultValue::Optional(kind) = leaf.default {
            assert!(matches!(
                kind,
                OptionalKind::Int
                    | OptionalKind::Str
                    | OptionalKind::Float
                    | OptionalKind::Bool
                    | OptionalKind::List
            ));
        }
    }
}

#[test]
fn the_help_command_is_consistent() {
    build_config::command().debug_assert();
}

// Ported: merge_json.

#[test]
fn merge_preserves_old_values() {
    let (result, report) = merged(
        r#"{"key": "old_value", "nested": {"inner": "old_inner"}}"#,
        r#"{"key": "new_value", "nested": {"inner": "new_inner"}}"#,
        None,
    );
    assert_eq!(
        result,
        json(r#"{"key": "old_value", "nested": {"inner": "old_inner"}}"#)
    );
    assert_eq!(report, MergeReport::default());
}

#[test]
fn merge_adds_new_keys() {
    let (result, report) = merged(
        r#"{"existing": "value"}"#,
        r#"{"existing": "new_value", "new_key": "new_value", "nested": {"inner": "value"}}"#,
        None,
    );
    assert_eq!(
        result,
        json(r#"{"existing": "value", "new_key": "new_value", "nested": {"inner": "value"}}"#)
    );
    assert_eq!(report.added, ["new_key", "nested"]);
}

#[test]
fn merge_keeps_old_keys_by_default() {
    let (result, report) = merged(
        r#"{"keep_me": "value", "shared": "old"}"#,
        r#"{"shared": "new"}"#,
        None,
    );
    assert_eq!(get(&result, "keep_me"), Some(&string("value")));
    assert!(report.removed.is_empty());
}

#[test]
fn merge_removes_all_old_keys_with_empty_list() {
    let (result, report) = merged(
        r#"{"remove_me": "value", "shared": "old"}"#,
        r#"{"shared": "new"}"#,
        Some(&[]),
    );
    assert_eq!(keys(&result), ["shared"]);
    assert_eq!(report.removed, ["remove_me"]);
}

#[test]
fn merge_nested_objects() {
    let (result, report) = merged(
        r#"{"level1": {"level2": {"old_value": 100, "old_only": "keep"}}}"#,
        r#"{"level1": {"level2": {"old_value": 200, "new_only": "added"}}}"#,
        None,
    );
    assert_eq!(
        result,
        json(
            r#"{"level1": {"level2": {"old_value": 100, "old_only": "keep", "new_only": "added"}}}"#
        )
    );
    assert_eq!(report.added, ["level1.level2.new_only"]);
}

#[test]
fn merge_preserves_key_order() {
    let (result, _) = merged(
        r#"{"z_first": 1, "a_second": 2, "m_third": 3}"#,
        r#"{"a_second": 20, "m_third": 30, "z_first": 10, "new_last": 40}"#,
        None,
    );
    assert_eq!(
        keys(&result),
        ["z_first", "a_second", "m_third", "new_last"]
    );
}

#[test]
fn merge_report_tracks_added() {
    let (_, report) = merged(
        r#"{"existing": "value"}"#,
        r#"{"existing": "new", "added1": "v1", "added2": {"nested": "v2"}}"#,
        None,
    );
    assert_eq!(report.added, ["added1", "added2"]);
}

#[test]
fn merge_report_tracks_removed() {
    let (_, report) = merged(
        r#"{"keep": "v1", "remove1": "v2", "remove2": {"nested": "v3"}}"#,
        r#"{"keep": "new"}"#,
        Some(&[]),
    );
    assert_eq!(report.removed, ["remove1", "remove2"]);
}

#[test]
fn merge_with_empty_old() {
    let new = r#"{"key1": "value1", "key2": {"nested": "value2"}}"#;
    let (result, report) = merged("{}", new, None);
    assert_eq!(result, json(new));
    assert_eq!(report.added.len(), 2);
}

#[test]
fn merge_with_empty_new() {
    let old = r#"{"key1": "value1", "key2": {"nested": "value2"}}"#;
    assert_eq!(merged(old, "{}", None).0, json(old));
    let (result, report) = merged(old, "{}", Some(&[]));
    assert_eq!(result, json("{}"));
    assert_eq!(report.removed.len(), 2);
}

#[test]
fn merge_type_change_dict_to_scalar_keeps_the_dict() {
    let (result, _) = merged(
        r#"{"key": {"nested": "value"}}"#,
        r#"{"key": "scalar"}"#,
        None,
    );
    assert_eq!(result, json(r#"{"key": {"nested": "value"}}"#));
}

#[test]
fn merge_type_change_scalar_to_dict_takes_the_dict() {
    let (result, _) = merged(
        r#"{"key": "scalar"}"#,
        r#"{"key": {"nested": "value"}}"#,
        None,
    );
    assert_eq!(result, json(r#"{"key": {"nested": "value"}}"#));
}

#[test]
fn merge_with_lists_keeps_the_old_list() {
    let (result, _) = merged(
        r#"{"actions": ["route", "locate", "custom"]}"#,
        r#"{"actions": ["route", "locate", "height"]}"#,
        None,
    );
    assert_eq!(
        result,
        json(r#"{"actions": ["route", "locate", "custom"]}"#)
    );
}

#[test]
fn merge_realistic_valhalla_config() {
    let (result, report) = merged(
        r#"{"mjolnir": {"tile_dir": "/custom/path/tiles", "max_cache_size": 2000000000,
            "logging": {"type": "file", "file_name": "/var/log/valhalla.log"}},
            "loki": {"service_defaults": {"radius": 10}}}"#,
        r#"{"mjolnir": {"tile_dir": "/data/valhalla", "max_cache_size": 1000000000,
            "new_setting": true, "logging": {"type": "std_out", "color": true}},
            "loki": {"service_defaults": {"radius": 0, "new_default": 50}},
            "thor": {"logging": {"type": "std_out"}}}"#,
        None,
    );
    assert_eq!(
        get(&result, "mjolnir.tile_dir"),
        Some(&string("/custom/path/tiles"))
    );
    assert_eq!(
        get(&result, "mjolnir.max_cache_size"),
        Some(&int("2000000000"))
    );
    assert_eq!(get(&result, "mjolnir.logging.type"), Some(&string("file")));
    assert_eq!(
        get(&result, "loki.service_defaults.radius"),
        Some(&int("10"))
    );
    assert_eq!(
        get(&result, "mjolnir.new_setting"),
        Some(&Value::Bool(true))
    );
    assert_eq!(
        get(&result, "mjolnir.logging.color"),
        Some(&Value::Bool(true))
    );
    assert_eq!(
        get(&result, "loki.service_defaults.new_default"),
        Some(&int("50"))
    );
    assert_eq!(get(&result, "thor.logging.type"), Some(&string("std_out")));
    assert!(report.added.contains(&"mjolnir.new_setting".to_owned()));
    assert!(report.added.contains(&"thor".to_owned()));
}

#[test]
fn merge_removes_specific_keys_only() {
    let (result, report) = merged(
        r#"{"keep_me": "value", "remove_me": "value", "also_keep": "value",
            "nested": {"keep": "inner", "remove": "inner"}}"#,
        r#"{"shared": "new", "nested": {"keep": "new_inner"}}"#,
        Some(&["remove_me", "nested.remove"]),
    );
    assert_eq!(keys(&result), ["keep_me", "also_keep", "nested", "shared"]);
    assert_eq!(keys(get(&result, "nested").unwrap()), ["keep"]);
    assert_eq!(report.removed, ["remove_me", "nested.remove"]);
}

#[test]
fn merge_removes_nested_subtree() {
    let (result, report) = merged(
        r#"{"keep": "value", "remove_subtree": {"child1": "value1", "child2": {"grandchild": "value2"}}}"#,
        r#"{"shared": "new"}"#,
        Some(&["remove_subtree"]),
    );
    assert_eq!(keys(&result), ["keep", "shared"]);
    assert_eq!(report.removed, ["remove_subtree"]);
}

#[test]
fn merge_remove_keys_does_not_affect_existing_in_new() {
    let (result, report) = merged(
        r#"{"both": "old_value", "old_only": "value"}"#,
        r#"{"both": "new_value"}"#,
        Some(&["both", "old_only"]),
    );
    assert_eq!(result, json(r#"{"both": "old_value"}"#));
    assert_eq!(report.removed, ["old_only"]);
}

// Ported: should_remove_key.

#[test]
fn should_remove_key_none_removes_nothing() {
    assert!(!should_remove_key("any.key", None));
}

#[test]
fn should_remove_key_empty_list_removes_all() {
    assert!(should_remove_key("any.key", Some(&[])));
}

#[test]
fn should_remove_key_exact_match() {
    assert!(should_remove_key(
        "mjolnir.logging",
        Some(&["mjolnir.logging".into()])
    ));
}

#[test]
fn should_remove_key_child_of_specified_path() {
    assert!(should_remove_key(
        "mjolnir.logging.type",
        Some(&["mjolnir.logging".into()])
    ));
}

#[test]
fn should_remove_key_no_match() {
    assert!(!should_remove_key(
        "loki.service",
        Some(&["mjolnir.logging".into()])
    ));
}

#[test]
fn should_remove_key_partial_match_not_removed() {
    assert!(!should_remove_key(
        "mjolnir.log",
        Some(&["mjolnir.logging".into()])
    ));
}

// The argparse command line.

#[test]
fn options_accept_the_equals_form() {
    assert_eq!(
        leaf_value(&["--mjolnir-tile-dir=/a=b"], "mjolnir-tile-dir"),
        Some(string("/a=b"))
    );
}

#[test]
fn options_accept_unambiguous_prefixes() {
    assert_eq!(
        leaf_value(&["--mjolnir-tile-d", "/t"], "mjolnir-tile-dir"),
        Some(string("/t"))
    );
}

#[test]
fn an_exact_option_wins_over_longer_ones() {
    assert_eq!(
        leaf_value(&["--mjolnir-tile-url", "u"], "mjolnir-tile-url"),
        Some(string("u"))
    );
}

#[test]
fn ambiguous_prefixes_are_refused() {
    assert!(matches!(
        parsed(&["--mjolnir-tile"]),
        Err(UsageError::Ambiguous { .. })
    ));
}

#[test]
fn negative_numbers_are_values() {
    assert_eq!(
        leaf_value(
            &["--httpd-service-timeout-seconds", "-5"],
            "httpd-service-timeout-seconds"
        ),
        Some(int("-5"))
    );
}

#[test]
fn hyphen_values_with_spaces_are_values() {
    assert_eq!(
        leaf_value(&["--mjolnir-tile-dir", "-a b"], "mjolnir-tile-dir"),
        Some(string("-a b"))
    );
}

#[test]
fn option_like_values_are_refused() {
    assert_eq!(
        parsed(&["--mjolnir-tile-dir", "-inf"]),
        Err(UsageError::ExpectedOne("--mjolnir-tile-dir".into()))
    );
}

#[test]
fn a_repeated_option_keeps_the_last_value() {
    assert_eq!(
        leaf_value(
            &["--statsd-tags", "a", "b", "--statsd-tags", "c"],
            "statsd-tags"
        ),
        Some(Value::List(vec![string("c")]))
    );
}

#[test]
fn short_flags_cluster() {
    let arguments = arguments(&["-pr"]);
    assert!(arguments.report);
    assert_eq!(arguments.remove_old, Some(Vec::new()));
}

#[test]
fn a_short_option_takes_an_attached_value() {
    assert_eq!(arguments(&["-rp"]).remove_old, Some(vec!["p".to_owned()]));
    assert_eq!(
        arguments(&["-mfile.json"]).merge_config,
        Some("file.json".to_owned())
    );
}

#[test]
fn a_flag_with_an_explicit_value_is_refused() {
    assert!(matches!(
        parsed(&["--report=1"]),
        Err(UsageError::IgnoredExplicit { .. })
    ));
}

#[test]
fn remove_old_is_none_when_absent() {
    assert_eq!(arguments(&[]).remove_old, None);
}

#[test]
fn stray_arguments_are_refused() {
    assert_eq!(
        parsed(&["stray", "--", "x"]),
        Err(UsageError::Unrecognized("stray -- x".into()))
    );
}

#[test]
fn help_is_shown_unless_an_earlier_argument_fails() {
    assert_eq!(parsed(&["--bogus", "-h"]), Ok(Parsed::Help));
    assert!(parsed(&["--logging-color", "x", "-h"]).is_err());
}

// The binary.

#[test]
fn without_arguments_the_defaults_are_written_like_json_dumps() {
    let dir = TempDir::new().unwrap();
    let output = run(dir.path(), &[]);
    assert_eq!(output.status.code(), Some(0));
    let text = stdout(&output);
    assert!(
        text.starts_with("{\n  \"logging\": {\n    \"type\": \"std_out\",\n    \"color\": true,")
    );
    assert!(text.contains("\"expand_within_distance\": {\n          \"0\": 100000000.0,"));
    assert!(text.contains("\"proxy\": \"ipc:///tmp/loki\""));
    assert!(text.ends_with("\"batch_size\": 500\n  }\n}\n"));
}

#[test]
fn non_ascii_values_are_escaped() {
    let dir = TempDir::new().unwrap();
    let output = run(dir.path(), &["--mjolnir-tile-dir", "é😀\u{7f}/"]);
    assert!(stdout(&output).contains(r#""tile_dir": "\u00e9\ud83d\ude00\u007f/","#));
}

#[test]
fn values_follow_python_number_rules() {
    let config = generated(&[
        "--mjolnir-max-cache-size",
        " 1_000_000_000_000_000_000_000 ",
        "--meili-default-sigma-z",
        "1e400",
        "--meili-default-beta",
        "5",
    ]);
    assert_eq!(
        get(&config, "mjolnir.max_cache_size"),
        Some(&int("1000000000000000000000"))
    );
    assert_eq!(
        get(&config, "meili.default.sigma_z"),
        Some(&Value::Float(f64::INFINITY))
    );
    assert_eq!(get(&config, "meili.default.beta"), Some(&int("5")));
}

#[test]
fn usage_errors_exit_2() {
    let dir = TempDir::new().unwrap();
    let output = run(dir.path(), &["--mjolnir-max-cache-size", "1.5"]);
    assert_eq!(output.status.code(), Some(2));
    assert!(stderr(&output).contains("invalid int value"));
    assert!(output.stdout.is_empty());
}

#[test]
fn help_goes_to_stdout_with_each_option_s_help_and_default() {
    let dir = TempDir::new().unwrap();
    let output = run(dir.path(), &["-h"]);
    assert_eq!(output.status.code(), Some(0));
    assert!(stdout(&output).contains("--mjolnir-tile-dir <MJOLNIR_TILE_DIR>"));
    assert!(stdout(&output).contains("[default: /data/valhalla]"));
}

#[test]
fn report_without_merge_warns() {
    let dir = TempDir::new().unwrap();
    let output = run(dir.path(), &["-p"]);
    assert_eq!(output.status.code(), Some(0));
    assert_eq!(stderr(&output), "Warn: --report requires --merge-config\n");
}

#[test]
fn merge_keeps_existing_values_and_reports_on_stderr() {
    let dir = TempDir::new().unwrap();
    fs::write(
        dir.path().join("old.json"),
        r#"{"mjolnir": {"tile_dir": "/mine", "old": 1.50, "big": 123456789012345678901234567890},
            "zzz": {"x": 1e3}, "zzz": {"y": -0.0}}"#,
    )
    .unwrap();
    let output = run(dir.path(), &["-m", "old.json", "-p", "-r", "mjolnir.big"]);
    assert_eq!(output.status.code(), Some(0));
    let config = build_config::parse(stdout(&output)).unwrap();
    assert_eq!(get(&config, "mjolnir.tile_dir"), Some(&string("/mine")));
    assert_eq!(keys(&config)[..2], ["mjolnir", "zzz"]);
    assert_eq!(get(&config, "zzz"), Some(&json(r#"{"y": -0.0}"#)));
    assert!(stdout(&output).contains("\"old\": 1.5,"));
    let report = stderr(&output);
    assert!(report.starts_with("\nAdded (from new config):\n  + additional_data\n  + httpd\n"));
    assert!(report.ends_with("\nRemoved (not in new config):\n  - mjolnir.big\n"));
}

#[test]
fn merge_of_the_defaults_reports_no_difference() {
    let dir = TempDir::new().unwrap();
    fs::write(dir.path().join("old.json"), run(dir.path(), &[]).stdout).unwrap();
    let output = run(dir.path(), &["-m", "old.json", "-p"]);
    assert_eq!(stderr(&output), "\nNo differences found.\n");
}

#[test]
fn a_missing_merge_config_exits_1() {
    let dir = TempDir::new().unwrap();
    let output = run(dir.path(), &["-m", "./a//missing.json"]);
    assert_eq!(output.status.code(), Some(1));
    assert_eq!(
        stderr(&output),
        "Error: Config file not found: a/missing.json\n"
    );
}

#[test]
fn an_invalid_merge_config_exits_1_with_python_s_message() {
    let dir = TempDir::new().unwrap();
    fs::write(dir.path().join("bad.json"), "{\"a\": 1,}").unwrap();
    let output = run(dir.path(), &["-m", "bad.json"]);
    assert_eq!(output.status.code(), Some(1));
    assert_eq!(
        stderr(&output),
        "Error: Invalid JSON in config: Illegal trailing comma before end of object: line 1 column 8 (char 7)\n"
    );
}

#[test]
fn an_unreadable_merge_config_exits_1() {
    let dir = TempDir::new().unwrap();
    let output = run(dir.path(), &["-m", ""]);
    assert_eq!(output.status.code(), Some(1));
    assert!(stderr(&output).starts_with("Error: cannot read .: "));
}

#[test]
fn a_merge_config_that_is_not_utf8_exits_1() {
    let dir = TempDir::new().unwrap();
    fs::write(dir.path().join("latin1.json"), b"{\"a\": \"\xe9\"}").unwrap();
    let output = run(dir.path(), &["-m", "latin1.json"]);
    assert_eq!(output.status.code(), Some(1));
    assert!(stderr(&output).starts_with("Error: latin1.json is not valid UTF-8"));
}

#[test]
fn output_writes_the_file_and_says_so() {
    let dir = TempDir::new().unwrap();
    let output = run(dir.path(), &["-o", "./out.json/"]);
    assert_eq!(output.status.code(), Some(0));
    assert!(output.stdout.is_empty());
    assert_eq!(stderr(&output), "Config written to: out.json\n");
    let written = fs::read(dir.path().join("out.json")).unwrap();
    assert_eq!(written, run(dir.path(), &[]).stdout);
}

#[test]
fn an_unwritable_output_exits_1() {
    let dir = TempDir::new().unwrap();
    let output = run(dir.path(), &["-o", "missing/out.json"]);
    assert_eq!(output.status.code(), Some(1));
    assert!(stderr(&output).starts_with("Error: cannot write missing/out.json: "));
}

fn nested_objects(depth: usize) -> String {
    "{\"a\": ".repeat(depth) + "1" + &"}".repeat(depth)
}

#[test]
fn merge_configs_nest_up_to_the_depth_limit() {
    let dir = TempDir::new().unwrap();
    fs::write(
        dir.path().join("deep.json"),
        nested_objects(build_config::MAX_DEPTH),
    )
    .unwrap();
    let output = run(dir.path(), &["-m", "deep.json"]);
    assert_eq!(output.status.code(), Some(0), "{}", stderr(&output));
}

#[test]
fn merge_configs_deeper_than_the_limit_exit_1() {
    let dir = TempDir::new().unwrap();
    let depth = build_config::MAX_DEPTH + 1;
    fs::write(dir.path().join("deep.json"), nested_objects(depth)).unwrap();
    let output = run(dir.path(), &["-m", "deep.json"]);
    assert_eq!(output.status.code(), Some(1));
    let position = 6 * build_config::MAX_DEPTH;
    assert_eq!(
        stderr(&output),
        format!(
            "Error: Invalid JSON in config: nesting deeper than {} levels is not supported: \
             line 1 column {} (char {position})\n",
            build_config::MAX_DEPTH,
            position + 1
        )
    );
}
