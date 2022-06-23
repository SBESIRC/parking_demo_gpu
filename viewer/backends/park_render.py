import time

import pygame
from pygame.locals import (QUIT, KEYDOWN, KEYUP, MOUSEBUTTONDOWN,
                           MOUSEBUTTONUP, MOUSEMOTION, KMOD_LSHIFT)
from pyphysx_utils.rate import Rate
import pkphysx as px
from pkphysx import (RigidActor, RigidDynamic, RigidStatic, GeometryType)
import numpy as np
import quaternion
from ..settings import fwSettings
from parking.UserData import *
from parking.utils import *


class ParkRender(object):

    colors = {
        "RigidStatic": (255, 255, 255, 255),
        "RigidDynamic": (127, 127, 127, 70),
        "RigidActor": (127, 127, 230, 255),
        "LMouse": (255, 0, 0, 255),
        "RMouse": (0, 255, 0, 255),
        "SelectedActor":(255,255,0,255),
        "LaneNode":(125,255,125,255),
        "aabbActor":(0,255,255,255)
    }

    def __init__(self, screen_width, screen_height, viewCenter, viewZoom, framework=None):
        self.SCREEN_WIDTH = screen_width
        self.SCREEN_HEIGHT = screen_height
        self.viewCenter = viewCenter
        self.PPM = viewZoom

        self.mouse_p = None # 记录鼠标位置（world）
        self.mouse_p_flag = "LMouse"
        self.selectedIndex= None
        self.framework = framework


        self.SCREEN_OFFSETX, self.SCREEN_OFFSETY = self.SCREEN_WIDTH * 1.0 / 2 - \
            self.viewCenter[0]*self.PPM, self.SCREEN_HEIGHT * \
            1.0/2+self.viewCenter[1]*self.PPM

    def updatePPM(self, viewZoom):
        self.PPM = viewZoom
        self.updateOffset()

    def updateCenter(self, center):
        self.viewCenter = center
        self.updateOffset()

    def setOffset(self, offset):
        self.SCREEN_OFFSETX = offset[0]
        self.SCREEN_OFFSETY = offset[1]

    def updateOffset(self):
        self.SCREEN_OFFSETX = self.SCREEN_WIDTH * \
            1.0 / 2-self.viewCenter[0]*self.PPM
        self.SCREEN_OFFSETY = self.SCREEN_HEIGHT * \
            1.0/2+self.viewCenter[1]*self.PPM

    def fix_vertices(self, vertices):
        """ 
        vertices:list(tuple)
        world->screen
        """
        return [(self.SCREEN_OFFSETX + self.PPM*v[0], self.SCREEN_OFFSETY - self.PPM*v[1]) for v in vertices]
        # return [(int(self.SCREEN_OFFSETX + self.PPM*(v[0]-self.viewCenter[0])), int(self.SCREEN_OFFSETY - self.PPM*(v[1]-self.viewCenter[1]))) for v in vertices]

    def get_global_location(self,local_pos,pose):
        # print(local_pos)
        # p=quaternion.quaternion(0,local_pos[0],local_pos[1],local_pos[2])
        w,a,b,c=quaternion.as_float_array(pose[1])
        q=quaternion.quaternion(w,a,b,c)
        m=quaternion.as_rotation_matrix(q)
        x= np.dot(m,local_pos)+pose[0]
        # print(x)
        return x[0:2]

    def _draw_test(self, screen):
        # 测试：画一下定位点
        v = ([10, 10], [10, 20], [20, 20], [20, 10])
        pygame.draw.polygon(screen, (255, 0, 0, 255), v, 1)

        v = ([700, 500], [710, 500], [710, 510], [700, 510])
        pygame.draw.polygon(screen, (0, 255, 0, 255), v, 1)

    def _draw_mouse_motion(self, point, screen):
        """
        mouse_shapes = self.framework.aabb_actor.get_atached_shapes()
        for mouse_shape in mouse_shapes:
            self._draw_box(mouse_shape, screen, self.framework.aabb_actor, "aabbActor")
        if self.framework.mouseJoint is not None:
            a0_pose = self.framework.mouseJoint.get_local_pose(0)
            a1_pose = self.framework.mouseJoint.get_local_pose(1)
            # print(self.framework.mouseActor.get_global_pose(), self.framework.selectedActor.get_global_pose())
            # print(a0_pose, a1_pose)
        """
        if point is None:
            return
        offset_x, offset_y = 1, 1

        origin_vertices = ([point[0]+offset_x, point[1]+offset_y],
                           [point[0]+offset_x, point[1]-offset_y],
                           [point[0]-offset_x, point[1]-offset_y],
                           [point[0]-offset_x, point[1]+offset_y])
        vertices = self.fix_vertices(origin_vertices)

        # vertices=origin_vertices
        pygame.draw.polygon(
            screen, [c / 2.0 for c in self.colors[self.mouse_p_flag]], vertices, 0)  # width=0
        pygame.draw.polygon(screen, self.colors[self.mouse_p_flag], vertices, 1)  # width=1

    def _draw_box(self, box, screen, actor, flag):
        """
        绘制box
        """
        if actor.get_user_data().Type=="LaneNode" and flag !="SelectedActor":
            flag="LaneNode"


        global_pose = actor.get_global_pose()
        vs = box.get_box_half_extents()
        origin_vertices=([vs[0],vs[1],vs[2]],
                            [vs[0],-vs[1],vs[2]],
                            [-vs[0],-vs[1],vs[2]],
                            [-vs[0],vs[1],vs[2]])
        vertices=[self.get_global_location(v,global_pose) for v in origin_vertices]
        # origin_vertices = ([vs[0]+offset_x, vs[1]+offset_y],
        #                    [vs[0]+offset_x, -vs[1]+offset_y],
        #                    [-vs[0]+offset_x, -vs[1]+offset_y],
        #                    [-vs[0]+offset_x, vs[1]+offset_y])
        vertices = self.fix_vertices(vertices)

        # vertices=origin_vertices
        pygame.draw.polygon(
            screen, [c / 2.0 for c in self.colors[flag]], vertices, 0)  # width=0
        pygame.draw.polygon(screen, self.colors[flag], vertices, 1)  # width=1

    def _draw_mesh(self, mesh, screen, actor, flag):
        
        global_pose = actor.get_global_pose()
        

        # print("draw mesh-global:")
        # print(offset_x)

        faces = mesh.get_shape_data()
        face_num = faces.shape[0]

        for i in range(face_num):
            # print(faces[i])
            v1 = (faces[i][0], faces[i][1],faces[i][2])
            v2 = (faces[i][3], faces[i][4],faces[i][5])
            v3 = (faces[i][6], faces[i][7],faces[i][8])

            origin_vertices = (v1, v2, v3)
            vertices=[self.get_global_location(v,global_pose) for v in origin_vertices]
            vertices = self.fix_vertices(vertices)
            # vertices=origin_vertices
            # print(vertices)
            pygame.draw.polygon(screen, self.colors[flag], vertices, 1)

    def draw_scene(self, screen, balancer):
        # Draw the world

        scene = balancer.scene
        # self._draw_test(screen)
        
        actors = scene.get_static_rigid_actors()

        for one_actor in actors:
            shapes = one_actor.get_atached_shapes()
            # print("shape size:{}".format(len(shapes)))
            for one_shape in shapes:
                geo_type = one_shape.get_geometry_type()
                if geo_type == GeometryType.BOX:
                    self._draw_box(one_shape, screen, one_actor, "RigidStatic")
                elif geo_type == GeometryType.TRIANGLEMESH:
                    self._draw_mesh(one_shape, screen,
                                    one_actor, "RigidStatic")
                else:
                    pass
                    # print(geo_type)

        actors = scene.get_dynamic_rigid_actors()
        i=0
        
        for one_actor in actors:
            flag="RigidDynamic"
            shapes = one_actor.get_atached_shapes()
            if self.selectedIndex==i:
                flag="SelectedActor"
            # print("shape size:{}".format(len(shapes)))
            for one_shape in shapes:
                geo_type = one_shape.get_geometry_type()
                if geo_type == GeometryType.BOX:
                    self._draw_box(one_shape, screen,
                                   one_actor, flag)
                elif geo_type == GeometryType.TRIANGLEMESH:
                    self._draw_mesh(one_shape, screen,
                                    one_actor, flag)
                else:
                    pass
                    # print(geo_type)
            i+=1
        self._draw_mouse_motion(self.mouse_p, screen)

