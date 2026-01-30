import unittest
from unittest.mock import MagicMock

import valhalla_build_config

# mock the add_argument so we can track which arguments it's called with in the main code
mock_parser = valhalla_build_config.parser
mock_parser.add_argument = MagicMock()


class TestBuildConfig(unittest.TestCase):
    def test_add_leaf_types(self):
        # test all data types, except for bool & list
        # they have custom lambdas we can't test this way
        help_text = {
            "str": "string",
            "int": "integer",
            "float": "float",
            "opt_str": "optional string",
            "opt_list": "optional list",
            "opt_int": "optional int",
        }
        config = {
            "str": "string",
            "int": 100,
            "float": 10.9,
            "opt_str": valhalla_build_config.Optional(str),
            "opt_list": valhalla_build_config.Optional(list),
            "opt_int": valhalla_build_config.Optional(int),
        }

        # these are the types the config values should resolve to
        # for the parser.add_argument(type=) arg
        value_types = {
            "str": str,
            "int": int,
            "float": float,
            "opt_str": str,
            "opt_list": list,
            "opt_int": int,
        }

        leaves = list()
        valhalla_build_config.add_leaf_args("", config, leaves, mock_parser, help_text)

        # make sure the parser.add_argument was called with the right stuff while parsing the config
        for name, default in config.items():
            if "list" in name:
                mock_parser.add_argument.assert_any_call(
                    f"--{name.replace('_', '-')}",
                    nargs="+",
                    help=help_text[name],
                    default=default,
                )
            else:
                mock_parser.add_argument.assert_any_call(
                    f"--{name.replace('_', '-')}",
                    type=value_types[name],
                    help=help_text[name],
                    default=default,
                )

    def test_nested_config(self):
        """tests if we get the values of nested dicts"""
        config = {
            "0": {
                "bool": False,
                "opt_str": valhalla_build_config.Optional(str),
                "1": {"list": [0, 1, 2]},
            }
        }
        help_text = {
            "0": {
                "bool": "boolean",
                "opt_str": "optional string",
                "1": {"list": "list"},
            }
        }

        # populate the leaves
        leaves = list()
        valhalla_build_config.add_leaf_args("", config, leaves, mock_parser, help_text)

        # 3 unique values
        self.assertEqual(len(set(leaves)), 3)

        expected_keys = ["0\x00bool", "0\x00opt_str", "0\x001\x00list"]
        self.assertEqual(leaves, expected_keys)

    def test_override_config(self):
        """tests end to end if the arg parsing is working"""
        config = {
            "0": {
                "bool": False,
                "opt_str": valhalla_build_config.Optional(str),
                "1": {"list": [0, 1, 2]},
            }
        }
        help_text = {
            "0": {
                "bool": "boolean",
                "opt_str": "optional string",
                "1": {"list": "list"},
            }
        }

        # populate the leaves
        leaves = list()
        valhalla_build_config.add_leaf_args("", config, leaves, mock_parser, help_text)

        # create the args mock
        args = {
            "0_bool": True,
            "0_opt_str": valhalla_build_config.Optional(str),
            "0_1_list": ["item1", "item2", "item3"],
        }

        valhalla_build_config.override_config(args, leaves, config)
        # the Optional arg should be removed
        self.assertEqual(len(config["0"]), 2)
        self.assertEqual(
            config, {"0": {"bool": True, "1": {"list": ["item1", "item2", "item3"]}}}
        )


class TestMergeJson(unittest.TestCase):
    """Test suite for merge_json function."""

    def test_merge_preserves_old_values(self):
        """Test that existing values are preserved during merge."""
        old = {"key": "old_value", "nested": {"inner": "old_inner"}}
        new = {"key": "new_value", "nested": {"inner": "new_inner"}}
        report = valhalla_build_config.MergeReport()

        result = valhalla_build_config.merge_json(old, new, report, remove_old=False)

        self.assertEqual(result["key"], "old_value")
        self.assertEqual(result["nested"]["inner"], "old_inner")
        self.assertEqual(len(report.added), 0)
        self.assertEqual(len(report.removed), 0)

    def test_merge_adds_new_keys(self):
        """Test that new keys are added with their new values."""
        old = {"existing": "value"}
        new = {"existing": "new_value", "new_key": "new_value", "nested": {"inner": "value"}}
        report = valhalla_build_config.MergeReport()

        result = valhalla_build_config.merge_json(old, new, report, remove_old=False)

        self.assertEqual(result["existing"], "value")  # preserved
        self.assertEqual(result["new_key"], "new_value")  # added
        self.assertEqual(result["nested"]["inner"], "value")  # added
        self.assertIn("new_key", report.added)
        self.assertIn("nested", report.added)

    def test_merge_keeps_old_keys_by_default(self):
        """Test that keys only in old are kept when remove_old=False."""
        old = {"keep_me": "value", "shared": "old"}
        new = {"shared": "new"}
        report = valhalla_build_config.MergeReport()

        result = valhalla_build_config.merge_json(old, new, report, remove_old=False)

        self.assertIn("keep_me", result)
        self.assertEqual(result["keep_me"], "value")
        self.assertEqual(len(report.removed), 0)

    def test_merge_removes_old_keys_with_flag(self):
        """Test that keys only in old are removed when remove_old=True."""
        old = {"remove_me": "value", "shared": "old"}
        new = {"shared": "new"}
        report = valhalla_build_config.MergeReport()

        result = valhalla_build_config.merge_json(old, new, report, remove_old=True)

        self.assertNotIn("remove_me", result)
        self.assertIn("remove_me", report.removed)

    def test_merge_nested_objects(self):
        """Test recursive merging of nested objects."""
        old = {
            "level1": {
                "level2": {
                    "old_value": 100,
                    "old_only": "keep",
                }
            }
        }
        new = {
            "level1": {
                "level2": {
                    "old_value": 200,
                    "new_only": "added",
                }
            }
        }
        report = valhalla_build_config.MergeReport()

        result = valhalla_build_config.merge_json(old, new, report, remove_old=False)

        self.assertEqual(result["level1"]["level2"]["old_value"], 100)  # preserved
        self.assertEqual(result["level1"]["level2"]["new_only"], "added")  # added
        self.assertEqual(result["level1"]["level2"]["old_only"], "keep")  # kept
        self.assertIn("level1.level2.new_only", report.added)

    def test_merge_preserves_key_order(self):
        """Test that key order from old config is preserved."""
        old = {"z_first": 1, "a_second": 2, "m_third": 3}
        new = {"a_second": 20, "m_third": 30, "z_first": 10, "new_last": 40}
        report = valhalla_build_config.MergeReport()

        result = valhalla_build_config.merge_json(old, new, report, remove_old=False)

        keys = list(result.keys())
        # Old keys should come first in their original order
        self.assertEqual(keys[:3], ["z_first", "a_second", "m_third"])
        # New key should be at the end
        self.assertEqual(keys[3], "new_last")

    def test_merge_report_tracks_added(self):
        """Test that MergeReport correctly tracks added keys."""
        old = {"existing": "value"}
        new = {"existing": "new", "added1": "v1", "added2": {"nested": "v2"}}
        report = valhalla_build_config.MergeReport()

        valhalla_build_config.merge_json(old, new, report, remove_old=False)

        self.assertEqual(len(report.added), 2)
        self.assertIn("added1", report.added)
        self.assertIn("added2", report.added)

    def test_merge_report_tracks_removed(self):
        """Test that MergeReport correctly tracks removed keys."""
        old = {"keep": "v1", "remove1": "v2", "remove2": {"nested": "v3"}}
        new = {"keep": "new"}
        report = valhalla_build_config.MergeReport()

        valhalla_build_config.merge_json(old, new, report, remove_old=True)

        self.assertEqual(len(report.removed), 2)
        self.assertIn("remove1", report.removed)
        self.assertIn("remove2", report.removed)

    def test_merge_with_empty_old(self):
        """Test merging when old config is empty."""
        old = {}
        new = {"key1": "value1", "key2": {"nested": "value2"}}
        report = valhalla_build_config.MergeReport()

        result = valhalla_build_config.merge_json(old, new, report, remove_old=False)

        self.assertEqual(result, new)
        self.assertEqual(len(report.added), 2)

    def test_merge_with_empty_new(self):
        """Test merging when new config is empty."""
        old = {"key1": "value1", "key2": {"nested": "value2"}}
        new = {}
        report = valhalla_build_config.MergeReport()

        # Without remove_old, all old keys are kept
        result = valhalla_build_config.merge_json(old, new, report, remove_old=False)
        self.assertEqual(result, old)

        # With remove_old, all old keys are removed
        report2 = valhalla_build_config.MergeReport()
        result2 = valhalla_build_config.merge_json(old, new, report2, remove_old=True)
        self.assertEqual(result2, {})
        self.assertEqual(len(report2.removed), 2)

    def test_merge_type_change_dict_to_scalar(self):
        """Test merging when type changes from dict to scalar."""
        old = {"key": {"nested": "value"}}
        new = {"key": "scalar"}
        report = valhalla_build_config.MergeReport()

        result = valhalla_build_config.merge_json(old, new, report, remove_old=False)

        # Old value (dict) is preserved since new is not a dict
        self.assertEqual(result["key"], {"nested": "value"})

    def test_merge_type_change_scalar_to_dict(self):
        """Test merging when type changes from scalar to dict."""
        old = {"key": "scalar"}
        new = {"key": {"nested": "value"}}
        report = valhalla_build_config.MergeReport()

        result = valhalla_build_config.merge_json(old, new, report, remove_old=False)

        # New value (dict) is used since we can't merge scalar into dict
        self.assertEqual(result["key"], {"nested": "value"})

    def test_merge_with_lists(self):
        """Test that list values are preserved from old config."""
        old = {"actions": ["route", "locate", "custom"]}
        new = {"actions": ["route", "locate", "height"]}
        report = valhalla_build_config.MergeReport()

        result = valhalla_build_config.merge_json(old, new, report, remove_old=False)

        # Old list is preserved
        self.assertEqual(result["actions"], ["route", "locate", "custom"])

    def test_merge_realistic_valhalla_config(self):
        """Test merging with realistic Valhalla config structure."""
        old = {
            "mjolnir": {
                "tile_dir": "/custom/path/tiles",
                "max_cache_size": 2000000000,
                "logging": {"type": "file", "file_name": "/var/log/valhalla.log"},
            },
            "loki": {
                "service_defaults": {"radius": 10},
            },
        }
        new = {
            "mjolnir": {
                "tile_dir": "/data/valhalla",
                "max_cache_size": 1000000000,
                "new_setting": True,
                "logging": {"type": "std_out", "color": True},
            },
            "loki": {
                "service_defaults": {"radius": 0, "new_default": 50},
            },
            "thor": {"logging": {"type": "std_out"}},
        }
        report = valhalla_build_config.MergeReport()

        result = valhalla_build_config.merge_json(old, new, report, remove_old=False)

        # Old values preserved
        self.assertEqual(result["mjolnir"]["tile_dir"], "/custom/path/tiles")
        self.assertEqual(result["mjolnir"]["max_cache_size"], 2000000000)
        self.assertEqual(result["mjolnir"]["logging"]["type"], "file")
        self.assertEqual(result["loki"]["service_defaults"]["radius"], 10)

        # New keys added
        self.assertTrue(result["mjolnir"]["new_setting"])
        self.assertEqual(result["mjolnir"]["logging"]["color"], True)
        self.assertEqual(result["loki"]["service_defaults"]["new_default"], 50)
        self.assertEqual(result["thor"]["logging"]["type"], "std_out")

        # Check report
        self.assertIn("mjolnir.new_setting", report.added)
        self.assertIn("thor", report.added)


if __name__ == "__main__":
    unittest.main()
