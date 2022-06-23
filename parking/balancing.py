import math
import time
import numpy as np
import quaternion
import pkphysx as px
from pyphysx_utils.rate import Rate
import pygame as pg

from pyphysx_render.pyrender import PyPhysxViewer
import pyrender
from pyphysx_render.pyrender_trackball import RoboticTrackball
from pyphysx_render.meshcat_render import MeshcatViewer

from .UserData import *
from .utils import *

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
    centroid = np.array([0.0, 0.0, 0.0])

    dtype = np.float64

    __instance = None
    def __new__(cls, *args, **kwargs):
        """Single Instance"""
        if cls.__instance is None:
            cls.__instance = super().__new__(cls)
        return cls.__instance

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

    def __init_obstacles__(self, outer_bound, walls):
        assert(len(outer_bound) > 0)
        material = px.Material(
            static_friction=0.1,
            dynamic_friction=0.1,
            restitution=0.5)
        # tmp
        bounds = px.RigidStatic()
        Sum, tot = np.zeros((3,)), 0.
        for bound in outer_bound:
            coords = trans_plist(bound['coordinates'], mm_to_m)
            for i in range(len(coords)-1):
                pts = np.array([[*coords[-1], 0.], [*coords[i], 0.], [*coords[i+1], 0.]], dtype=self.dtype)
                sz = np.linalg.norm(np.cross(pts[1]-pts[0], pts[2]-pts[0]))
                tot += sz
                Sum += sz * np.mean(pts, axis=0)
                shape = createStringShape(self, coords[i:i+2][::-1], material, is_polygon=False)
                shape.setup_simulation_filtering(FilterType.Bound.value, ~(FilterType.Bound.value))
                bounds.attach_shape(shape)
        self.centroid = Sum / tot
        self.scene.add_actor(bounds)

        #land = px.RigidStatic.create_plane(material=material, distance=wall_height)
        #land.set_global_pose((self.centroid, quaternion.quaternion(0, 1, 0, 1)))
        #self.scene.add_actor(land)

        # walls
        # TODO: walls do not have userData
        # TODO: 是否要移动它们的中心
        # TODO: filter mask
        for wall in walls:
            wall_actor = px.RigidStatic()
            coords = trans_plist(wall['coordinates'], mm_to_m)
            if wall['type'] == "LineString":
                shape = createStringShape(self, coords, material, is_polygon=False)
                shape.setup_simulation_filtering(FilterType.Wall.value, ~(FilterType.Wall.value))
                wall_actor.attach_shape(shape)
            elif wall['type'] == "MultiLineString":
                for _wall in coords:
                    shape = createStringShape(self, _wall, material, is_polygon=False)
                    shape.setup_simulation_filtering(FilterType.Wall.value, ~(FilterType.Wall.value))
                    wall_actor.attach_shape(shape)
            elif wall['type'] == "Polygon":
                for poly in coords:
                    shape = createStringShape(self, poly, material, is_polygon=True)
                    shape.setup_simulation_filtering(FilterType.Wall.value, ~(FilterType.Wall.value))
                    wall_actor.attach_shape(shape)
            else:
                raise Exception("Unknown wall type" + wall['type'])
            self.scene.add_actor(wall_actor)

    def __init_lanes__(self, lanes, config):
        # TODO
        w = self.lane_width
        v_material = px.Material(static_friction=0.1, dynamic_friction=0.1, restitution=0.5)
        vertex_shape = px.Shape.create_box((w, w, w), v_material, is_exclusive=False)
        vertex_shape.setup_simulation_filtering(FilterType.LaneNode.value, ~(FilterType.LaneNode.value | FilterType.LaneBody.value))

        for i, v in enumerate(lanes['vertices']):
            loc = np.array([*mm_to_m(v['location']), 0], dtype=self.dtype)
            v_actor = px.RigidDynamic()
            v_actor.attach_shape(vertex_shape)
            v_actor.set_global_pose(loc)
            v_data = LaneNodeUserData(v_actor, Id=i, adj=0, adj_joints=[])
            v_actor.set_user_data(v_data)

            lock_2d(v_actor)
            self.scene.add_actor(v_actor)
            self.vertices.append(v_actor)
        
        for e in lanes['edges']:
            v1 = self.vertices[int(e['endings'][0])]
            v2 = self.vertices[int(e['endings'][1])]
            # TODO: 是否能正常运行
            v1.get_user_data().adj += 1
            v2.get_user_data().adj += 1

        for e in lanes['edges']:
            v1 = self.vertices[int(e['endings'][0])]
            v2 = self.vertices[int(e['endings'][1])]
            loc1, _ = v1.get_global_pose()
            loc2, _ = v2.get_global_pose()
            axis = loc2 - loc1
            axis_len = np.linalg.norm(axis, 2)
            u_axis = axis / axis_len
            # cos_theta = u_axis[0]
            # sin_theta = u_axis[1]

            # 设置lane node Joints
            matrix = np.array([
                [u_axis[0], -u_axis[1], 0],
                [u_axis[1], u_axis[0], 0],
                [0, 0, 1]], dtype=self.dtype)
            q = quaternion.from_rotation_matrix(matrix)
            print(axis_len, u_axis, "q = ", q)

            joint = px.D6Joint(v1, v2, (axis/2, q), (-axis/2, q)) # local poses
            joint.set_motion(px.D6Axis.X, px.D6Motion.FREE)
            # joint.set_motion(px.D6Axis.SWING2, px.D6Motion.FREE)

            # joint.set_kinemic_projection(True, tolerance=0.1)
            # joint = px.D6Joint(v1, v2, (np.zeros_like(loc1), q), (np.zeros_like(loc2), q)) # local poses
            # TODO: joint.set_linear_limit_soft
            # TODO: joint.set_break_force
            # 设置车道actor
            lane_shape = px.Shape.create_box((axis_len-w, w, w), v_material, is_exclusive=False)
            if lane_shape.is_valid():
                lane_shape.setup_simulation_filtering(FilterType.LaneBody.value, ~(FilterType.LaneNode.value | FilterType.LaneBody.value))
                lane_actor = px.RigidDynamic()
                lane_actor.attach_shape(lane_shape)
                lane_actor.set_mass(axis_len)
                lane_actor.set_global_pose(((loc1 + axis/2), q))
                lane_data = LaneBodyUserData(lane_actor, A=v1, B=v2)
                lane_actor.set_user_data(lane_data)
                self.scene.add_actor(lane_actor)
                # TODO: filtering未起作用
                # lock
                lock_2d(lane_actor)
                # lane_actor.set_linear_velocity(np.concatenate([(np.random.rand(2)-.5) * 20, [0]]))
                lane_joint1 = px.D6Joint(v1, lane_actor, (np.zeros_like(loc1), q), (-axis/4, q))
                # lane_joint2 = px.D6Joint(v2, lane_actor, (np.zeros_like(loc2), q), (axis/4, q))
                # TODO: 基本确定是joint冲突的问题
                # lane_joint1 = px.D6Joint(v1, lane_actor, (np.zeros_like(loc1), q), [-axis_len/2, 0, 0])
                # lane_joint2 = px.D6Joint(v2, lane_actor, (np.zeros_like(loc2), q), [axis_len/2, 0, 0])
                lane_joint1.set_motion(px.D6Axis.X, px.D6Motion.FREE)
                # lane_joint2.set_motion(px.D6Axis.X, px.D6Motion.FREE)

                # lane_joint1.set_kinemic_projection(True, tolerance=1)
                # lane_joint2.set_kinemic_projection(True, tolerance=1)
            else:
                print("invalid shape:", loc1, loc2)
                pass
                # TODO: 节点太近的情况

    def __init__(self, outer_bound, wall, lane, config):
        super(Balancer, self).__init__()

        use_gpu = True
        if use_gpu:
            px.Physics.init_gpu()

        num_cpus = 4
        scene_flags = [px.SceneFlag.ENABLE_CCD]
        # px.Physics.set_num_cpu(num_cpus)
        self.scene = px.Scene(scene_flags=scene_flags) if not use_gpu else px.Scene(
            scene_flags=[*scene_flags, px.SceneFlag.ENABLE_PCM, px.SceneFlag.ENABLE_GPU_DYNAMICS, px.SceneFlag.ENABLE_STABILIZATION],
            broad_phase_type=px.BroadPhaseType.GPU,
            gpu_max_num_partitions=32, gpu_dynamic_allocation_scale=16.,
        )

        self.__init_config__(config)
        # The obstacles
        self.__init_obstacles__(outer_bound, wall)
        # The lane
        self.__init_lanes__(lane, config)

    def update(self, dt, **kwargs):
        self.scene.simulate(dt)


    def run(self):
        start_time = time.time()
        rate = Rate(2400)
        dev = True
        # dev = False
        if dev:
            render_scene = pyrender.scene.Scene()
            render_scene.bg_color = np.array([0.75] * 3)
            if render_scene.main_camera_node is None:
                cam = pyrender.PerspectiveCamera(yfov=np.deg2rad(60), aspectRatio=1.414, znear=0.005)
                cam_pose = np.eye(4)
                target = self.centroid
                cam_pose[:3, 3] = RoboticTrackball.spherical_to_cartesian(12., np.deg2rad(-45.), 0., target=target)
                cam_pose[:3, :3] = RoboticTrackball.look_at_rotation(eye=cam_pose[:3, 3], target=target, up=[0, 1, 1])
                nc = pyrender.Node(camera=cam, matrix=cam_pose)
                render_scene.add_node(nc)
                render_scene.main_camera_node = nc
            render = PyPhysxViewer(render_scene=render_scene, viewer_flags={
                # 'use_perspective_cam': False,
                'show_world_axis': True,
                'show_mesh_axes': True,
                'rotate': False,
                'rotate_rate': 10,
                'rotate_axis': np.array([0, 0.01, 1]),
                'plane_grid_num_of_lines': 10 * 2,
                'plane_grid_spacing': 1. * 5 / ratio
            })
            # render = MeshcatViewer(render_to_animation=True, animation_fps=10)
            render.add_physx_scene(self.scene)
        for _ in range((1 << 10)):
            self.update(rate.period())
            if _ & 0xff == 0xff:
                print("simulate time per 256 frames:", time.time() - start_time)
                start_time = time.time()
            if dev:
                render.update(blocking=True)
            # rate.sleep()
        end_time = time.time()
