import unittest
from unittest.mock import MagicMock

try:
    from test.scripts import helper
except (ModuleNotFoundError, ImportError):
    import helper

# get the module to test
valhalla_build_config = helper.import_module('valhalla_build_config')
# mock the add_argument so we can track which arguments it's called with in the main code
mock_parser = valhalla_build_config.parser
mock_parser.add_argument = MagicMock()


class TestBuildConfig(unittest.TestCase):
    def test_add_leaf_types(self):
        # test all data types, except for bool & list, since they have costum lambdas we can't test
        help_text = {
            'bool': "boolean",
            "str": "string",
            "list": "list",
            "opt_str": "optional string",
            "opt_list": "optional list",
            "opt_int": "optional int",
        }
        config = {
            'str': 'string',
            'opt_str': valhalla_build_config.Optional(str),
            'opt_list': valhalla_build_config.Optional(list),
            'opt_int': valhalla_build_config.Optional(int),
        }

        value_types = {
            "str": str,
            "opt_str": str,
            "opt_list": list,
            "opt_int": int
        }

        leaves = list()
        valhalla_build_config.add_leaf_args('', config, leaves, mock_parser, help_text)

        # make sure the parser.add_argument was called with the right stuff while parsing the config
        for name, default in config.items():
            mock_parser.add_argument.assert_any_call(
                f'--{name.replace("_", "-")}', type=value_types[name], help=help_text[name], default=default
            )

    def test_nested_config(self):
        """tests if we get the values of nested dicts"""
        config = {"0": {"bool": False, "opt_str": valhalla_build_config.Optional(str), "1": {"list": [0, 1, 2]}}}
        help_text = {"0": {"bool": "boolean", "opt_str": "optional string", "1": {"list": "list"}}}

        # populate the leaves
        leaves = list()
        valhalla_build_config.add_leaf_args('', config, leaves, mock_parser, help_text)

        # 3 unique values
        self.assertEqual(len(set(leaves)), 3)
        
        expected_keys = ["0\x00bool", "0\x00opt_str", "0\x001\x00list"]
        self.assertEqual(leaves, expected_keys)

    def test_override_config(self):
        """tests end to end if the arg parsing is working"""
        config = {"0": {"bool": False, "opt_str": valhalla_build_config.Optional(str), "1": {"list": [0, 1, 2]}}}
        help_text = {"0": {"bool": "boolean", "opt_str": "optional string", "1": {"list": "list"}}}

        # populate the leaves
        leaves = list()
        valhalla_build_config.add_leaf_args('', config, leaves, mock_parser, help_text)

        # create the args mock
        args = {'0_bool': True, "0_opt_str": valhalla_build_config.Optional(str), "0_1_list": [3, 4, 5]}

        valhalla_build_config.override_config(args, leaves, config)
        # the Optional arg should be removed
        self.assertTrue(len(config["0"]), 2)
        self.assertEqual(config, {"0": {"bool": True, "1": {"list": [3, 4, 5]}}})


if __name__ == '__main__':
    unittest.main()
