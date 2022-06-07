import dataloader
import argparse
from viewer import Framework

def test_viewer(ns: argparse.Namespace):
    filename = ns.file
    config_file = open(ns.config, "r+", encoding="utf-8")
    with open(filename, 'r') as f:
        lane_name = filename.rsplit('.', 1)[0]+'-lane.json' if ns.dump_lane else None
        origin, wall, outer_bound, lane = dataloader.load_geojson(f, lane_name)
    config = dataloader.load_config(config_file)
    
    
    # print(origin, wall, outer_bound, lane)
    # balancer = balancing.Balancer(outer_bound, wall, lane, config)
    # balancer.run()

