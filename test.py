import sys, os
import dataloader
import argparse
from parking.test import test

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--test', type=str, default='viewer', help='test name')
    parser.add_argument('-f', '--file', type=str, default='data/test/1.geojson')
    parser.add_argument('-j', '--dump_lane', type=bool, default=False)
    parser.add_argument('-c', '--config', type=str, default='data/config.json')
    parser.add_argument("--fwsettings", nargs="*", default=[])

    ns = parser.parse_args()
    ns.file = os.path.abspath(ns.file.replace('\\', '/'))
    ns.config = os.path.abspath(ns.config.replace('\\', '/'))
    if ns.test == 'test':
        test(ns)
    elif ns.test == 'viewer':
        sys.argv = [sys.argv[0]] + ns.fwsettings
        from viewer.test_viewer import test_viewer
        test_viewer(ns)
