import math
import time
import numpy as np
import pkphysx as px
from pyphysx_utils.rate import Rate
import pygame as pg

from pyphysx_render.pyrender import PyPhysxViewer

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
        # tmp
        land = px.RigidStatic.create_plane(material=material)
        self.scene.add_actor(land)
        dtype = np.float64
        height = 4.0
        bounds = px.RigidStatic()
        for bound in outer_bound:
            coords = trans_plist(bound['coordinates'], mm_to_m)
            n = len(coords) - 1
            vertices = np.empty((n << 1, 3), dtype=dtype)
            indices = np.empty((n << 1, 3), dtype=np.int32)
            for i in range(len(coords)-1):
                print(coords[i], coords[i+1])
                xa, xb = i<<1, (i+1)<<1
                xc, xd = xa+1, xb+1
                if i + 1 == n:
                    xb -= (n << 1)
                    xd -= (n << 1)
                print(xa, xb, xc, xd)
                #a, b = np.array([*coords[i], 0.], dtype=dtype), np.array([*coords[i+1], 0.], dtype=dtype)
                a, b = np.array([*coords[i], 0.], dtype=dtype)*.1, np.array([*coords[i+1], 0.], dtype=dtype)*.1
                c, d = a + [0, 0, height], b + [0, 0, height]
                vertices[xa:xa+2,:] = [a, c]
                indices[xa] = [xa, xc, xb]
                indices[xc] = [xc, xd, xb]

            shape = px.Shape.create_triangle_mesh(vertices, indices, material, is_exclusive=True, scale=1.)
            bounds.attach_shape(shape)
        self.scene.add_actor(bounds)

    def __init_lanes__(self, lane, config):
        pass

    def __init__(self, outer_bound, wall, lane, config):
        super(Balancer, self).__init__()

        use_gpu = True
        if use_gpu:
            px.Physics.init_gpu()

        num_cpus = 8
        px.Physics.set_num_cpu(num_cpus)
        self.scene = px.Scene() if not use_gpu else px.Scene(
            scene_flags=[px.SceneFlag.ENABLE_PCM, px.SceneFlag.ENABLE_GPU_DYNAMICS, px.SceneFlag.ENABLE_STABILIZATION],
            broad_phase_type=px.BroadPhaseType.GPU,
            gpu_max_num_partitions=32, gpu_dynamic_allocation_scale=16.,
        )

        self.__init_config__(config)
        # The obstacles
        self.__init_obstacles__(outer_bound, wall)
        # The lane
        self.__init_lanes__(lane, config)

    def run(self):
        start_time = time.time()
        rate = Rate(240)
        dev = True
        if dev:
            render = PyPhysxViewer()
            render.add_physx_scene(self.scene)
        for _ in range(1000):
            self.scene.simulate(rate.period())
            if dev:
                render.update()
            rate.sleep()
        end_time = time.time()
