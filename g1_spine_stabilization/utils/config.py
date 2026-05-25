"""YAML config loader.

Resolves a None path to the project's configs/default.yaml so callers
(scripts, tests) don't need to know about the layout.
"""
import os
import yaml


_PROJECT_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
)
DEFAULT_CONFIG_PATH = os.path.join(_PROJECT_ROOT, 'configs', 'default.yaml')


def load_config(path=None):
    """Load a YAML config file. Returns the parsed dict.

    If path is None, loads configs/default.yaml from the project root.
    """
    if path is None:
        path = DEFAULT_CONFIG_PATH
    with open(path, 'r') as f:
        return yaml.safe_load(f)
