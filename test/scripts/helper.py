from pathlib import Path
import sys
import importlib

CWD = Path(__file__).parent.resolve()


def import_path(module_name):
    """Returns the module as object"""
    # Get the right path to the module
    path = str(CWD.parent.parent.joinpath('scripts', module_name))
    spec = importlib.util.spec_from_loader(
        module_name,
        importlib.machinery.SourceFileLoader(module_name, path)
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    sys.modules[module_name] = module
    return module
