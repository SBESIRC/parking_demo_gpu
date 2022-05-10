import math
import numpy as np
import pyphysx as px
import pygame as pg


lane_ratio = 1000
graph_ratio = 1000
lane_ratio = 1000

def trans_plist(plist, f):
    if len(plist) == 0: return plist
    if type(plist[0]) == float:
        return f(plist)
    return [trans_plist(lst, f) for lst in plist]

def mm_to_m(p):
    return (p[0] / graph_ratio, p[1] / graph_ratio)

def m_to_mm(p):
    return (p[0] * graph_ratio, p[1] * graph_ratio)

def lane_trans(p):
    return (p[0] / lane_ratio, p[1] / lane_ratio)

def lane_trans_inv(p):
    return (p[0] * lane_ratio, p[1] * lane_ratio)

class Balancer:
    name = "Banlancer"
    description = "Balancer"
    wall_list = []
    vertices = []
    lane_joints = []
    lane_bodies = []
    spots = {}
    nxt_spot_id = 0
    config = {}

    def __init_config__(self, config):
        self.config = config
        self.query_range = config["query_range"] / lane_ratio
        self.spot_attract_range = config["spot_attract_range"] / lane_ratio
        self.parallel_spot = config["paralleled_spot"]
        for k, v in self.parallel_spot.items():
            self.parallel_spot[k] = v / lane_ratio
        self.vertical_spot = config["vertical_spot"]
        for k, v in self.vertical_spot.items():
            self.vertical_spot[k] = v / lane_ratio
        
        self.sampling_rate = config["sampling_rate"]
        self.force_rate = config['force_rate']['r']
        self.spots_force_rate = config['force_rate']['spots']
        self.step_id = 0
        self.lane_width = config['lane_width'] / lane_ratio
        self.base_threshold = config['remove_threshold']
        self.threshold_exp = [0.0, 0.0, 0.0]
        self.remove_thre_force = config['remove_threshold']['force']
        self.remove_thre_torque = config['remove_threshold']['torque']
        self.remove_thre_sin = config['remove_threshold']['sin'] * math.pi
        self.shake_impulse = config['shake_impulse']
        self.vibrate_speed = config['vibrate_speed']

    def __init_obstacles__(self, outer_bound, wall):
        assert(len(outer_bound) > 0)
        material = px.Material(
            static_friction=0.1,
            dynamic_friction=0.1,
            restitution=0.5)
        for bound in outer_bound:
            coords = trans_plist(bound['coordinates'], mm_to_m)
            for i in range(len(coords)-1):
                print(coords[i], coords[i+1])
            # ground = px.RigidStatic.create_plane( material=material,

    def __init_lanes__(self, lane, config):
        pass

    def __init__(self, outer_bound, wall, lane, config):
        super(Balancer, self).__init__()

        self.world = px.Scene()

        self.__init_config__(config)
        # The obstacles
        self.__init_obstacles__(outer_bound, wall)
        # The lane
        self.__init_lanes__(lane, config)

