"""Launch the bimanual pose-tracking sim controller."""

import argparse
import os
import sys

# Make the package importable when running this script directly without
# `pip install -e .`.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_THIS_DIR, '..'))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from g1_spine_stabilization.controllers.pose_tracking import run_pose_tracking_sim


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--config',
        default=None,
        help='Path to a YAML config (defaults to configs/default.yaml).',
    )
    args = parser.parse_args()

    run_pose_tracking_sim(config_path=args.config)


if __name__ == '__main__':
    main()
