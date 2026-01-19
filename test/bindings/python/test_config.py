# -*- coding: utf-8 -*-

import json
import os
import unittest
from pathlib import Path

from valhalla.config import parse_and_validate_config


class TestParseAndValidateConfig(unittest.TestCase):
    """Test suite for parse_and_validate_config function."""

    @classmethod
    def setUpClass(cls):
        """Set up test fixtures."""
        cls.tiles_path = Path("test/data/utrecht_tiles")
        # Use path relative to this test file
        cls.valid_config_path = Path(__file__).parent / "valhalla.json"
        cls.valid_config_dict = {
            "mjolnir": {
                "tile_dir": str(cls.tiles_path),
                "tile_extract": "test/data/utrecht_tiles/tiles.tar",
            }
        }

    def test_parse_dict_config(self):
        """Test parsing dict config creates temp file."""
        config_path, temp_file_path = parse_and_validate_config(self.valid_config_dict)

        # Should return tuple with temp file
        self.assertIsInstance(config_path, str)
        self.assertIsInstance(temp_file_path, str)
        self.assertEqual(config_path, temp_file_path)

        # Temp file should exist and contain valid JSON
        self.assertTrue(Path(temp_file_path).is_file())
        with open(temp_file_path) as f:
            parsed = json.load(f)
            self.assertEqual(parsed["mjolnir"]["tile_dir"], str(self.tiles_path))

        # Clean up temp file
        os.unlink(temp_file_path)

    def test_parse_dict_config_with_temp_file(self):
        """Test parsing dict config creates temp file (legacy test for compatibility)."""
        config_path, temp_file_path = parse_and_validate_config(self.valid_config_dict)

        # Should return tuple
        self.assertIsInstance(config_path, str)
        self.assertIsInstance(temp_file_path, str)
        self.assertEqual(config_path, temp_file_path)

        # Temp file should exist and contain valid JSON
        self.assertTrue(Path(temp_file_path).is_file())
        with open(temp_file_path) as f:
            parsed = json.load(f)
            self.assertEqual(parsed["mjolnir"]["tile_dir"], str(self.tiles_path))

        # Clean up temp file
        os.unlink(temp_file_path)

    def test_parse_json_string_config(self):
        """Test parsing JSON string config."""
        json_str = json.dumps(self.valid_config_dict)
        config_path, temp_file_path = parse_and_validate_config(json_str)

        # Should return the same JSON string with no temp file
        self.assertEqual(config_path, json_str)
        self.assertIsNone(temp_file_path)

    def test_parse_file_path_string(self):
        """Test parsing file path as string."""
        config_path, temp_file_path = parse_and_validate_config(str(self.valid_config_path))

        # Should return file path with no temp file
        self.assertEqual(config_path, str(self.valid_config_path))
        self.assertIsNone(temp_file_path)

    def test_parse_path_object(self):
        """Test parsing Path object."""
        config_path, temp_file_path = parse_and_validate_config(self.valid_config_path)

        # Should return resolved path string with no temp file
        self.assertIsInstance(config_path, str)
        self.assertTrue(Path(config_path).is_file())
        self.assertIsNone(temp_file_path)

    def test_invalid_type(self):
        """Test that invalid config types raise AttributeError."""
        with self.assertRaises(AttributeError) as exc:
            parse_and_validate_config(12345)
        self.assertIn("can't be of type", str(exc.exception))

        with self.assertRaises(AttributeError):
            parse_and_validate_config([1, 2, 3])

        with self.assertRaises(AttributeError):
            parse_and_validate_config(None)

    def test_nonexistent_file_path(self):
        """Test that non-existent file paths raise FileNotFoundError."""
        with self.assertRaises(FileNotFoundError) as exc:
            parse_and_validate_config("/nonexistent/config.json")
        self.assertIn("doesn't exist", str(exc.exception))

    def test_invalid_json_string(self):
        """Test that invalid JSON strings raise appropriate errors."""
        # Not a file and not valid JSON
        with self.assertRaises(FileNotFoundError):
            parse_and_validate_config("not a file path or json")

        # Invalid JSON syntax
        with self.assertRaises(json.JSONDecodeError):
            parse_and_validate_config('{"invalid": json}')

    def test_missing_mjolnir_section(self):
        """Test that config without mjolnir section raises AttributeError."""
        config = {"loki": {"actions": ["route"]}}

        with self.assertRaises(AttributeError) as exc:
            parse_and_validate_config(config)
        self.assertIn("mjolnir.tile_extract and mjolnir.tile_dir are missing", str(exc.exception))

    def test_missing_tile_extract_and_tile_dir(self):
        """Test that config without tile_extract AND tile_dir raises AttributeError."""
        config = {"mjolnir": {"logging": {"type": "std_out"}}}

        with self.assertRaises(AttributeError) as exc:
            parse_and_validate_config(config)
        self.assertIn("mjolnir.tile_extract and mjolnir.tile_dir are missing", str(exc.exception))

    def test_tile_data_not_exists(self):
        """Test that non-existent tile data raises FileNotFoundError."""
        config = {
            "mjolnir": {
                "tile_extract": "/nonexistent/tiles.tar",
                "tile_dir": "/nonexistent/tiles",
            }
        }

        with self.assertRaises(FileNotFoundError) as exc:
            parse_and_validate_config(config)
        self.assertIn("Can't load graph", str(exc.exception))

    def test_tile_dir_only(self):
        """Test config with only tile_dir (no tile_extract)."""
        config = {"mjolnir": {"tile_dir": str(self.tiles_path)}}

        config_path, temp_file_path = parse_and_validate_config(config)
        # Read from temp file
        with open(config_path) as f:
            parsed = json.load(f)
        self.assertEqual(parsed["mjolnir"]["tile_dir"], str(self.tiles_path))
        # Clean up
        if temp_file_path:
            os.unlink(temp_file_path)

    def test_tile_extract_only(self):
        """Test config with only tile_extract (no tile_dir)."""
        config = {"mjolnir": {"tile_extract": "test/data/utrecht_tiles/tiles.tar"}}

        config_path, temp_file_path = parse_and_validate_config(config)
        # Read from temp file
        with open(config_path) as f:
            parsed = json.load(f)
        self.assertEqual(parsed["mjolnir"]["tile_extract"], "test/data/utrecht_tiles/tiles.tar")
        # Clean up
        if temp_file_path:
            os.unlink(temp_file_path)

    def test_both_tile_sources_one_exists(self):
        """Test config with both tile_extract and tile_dir where only one exists."""
        # tile_extract doesn't exist, but tile_dir does - should succeed
        config = {
            "mjolnir": {
                "tile_extract": "/nonexistent/tiles.tar",
                "tile_dir": str(self.tiles_path),
            }
        }

        config_path, temp_file_path = parse_and_validate_config(config)
        # Read from temp file
        with open(config_path) as f:
            parsed = json.load(f)
        self.assertEqual(parsed["mjolnir"]["tile_dir"], str(self.tiles_path))
        # Clean up
        if temp_file_path:
            os.unlink(temp_file_path)

    def test_empty_tile_paths(self):
        """Test config with empty tile paths."""
        config = {"mjolnir": {"tile_extract": "", "tile_dir": ""}}

        with self.assertRaises(AttributeError) as exc:
            parse_and_validate_config(config)
        self.assertIn("mjolnir.tile_extract and mjolnir.tile_dir are missing", str(exc.exception))


if __name__ == "__main__":
    unittest.main()
