#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# C++ version Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
# Python version Copyright (c) 2010 kne / sirkne at gmail dot com
#
# This software is provided 'as-is', without any express or implied
# warranty.  In no event will the authors be held liable for any damages
# arising from the use of this software.
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
# 1. The origin of this software must not be misrepresented; you must not
# claim that you wrote the original software. If you use this software
# in a product, an acknowledgment in the product documentation would be
# appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be
# misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.

"""
A simple, minimal Pygame-based backend.
It will only draw and support very basic keyboard input (ESC to quit).

There are no main dependencies other than the actual test you are running.
Note that this only relies on framework.py for the loading of this backend,
and holding the Keys class. If you write a test that depends only on this
backend, you can remove references to that file here and import this module
directly in your test.

To use this backend, try:
 % python -m examples.web --backend simple

NOTE: Examples with Step() re-implemented are not yet supported, as I wanted
to do away with the Settings class. This means the following will definitely
not work: Breakable, Liquid, Raycast, TimeOfImpact, ... (incomplete)
"""
# NOTICE: 无控制面板
import time

import pygame
from pygame.locals import (QUIT, KEYDOWN, KEYUP, MOUSEBUTTONDOWN,
                           MOUSEBUTTONUP, MOUSEMOTION, KMOD_LSHIFT)
from pyphysx_utils.rate import Rate
import pkphysx as px
from pkphysx import (RigidActor, RigidDynamic, RigidStatic,GeometryType)
import numpy as np
from ..settings import fwSettings

TARGET_FPS = 60
PPM = 1.0  # scale
TIMESTEP = 1.0 / TARGET_FPS
VEL_ITERS, POS_ITERS = 10, 10
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 600
SCREEN_OFFSETX, SCREEN_OFFSETY = SCREEN_WIDTH * 1.0 / 4.0, SCREEN_HEIGHT * 3.0 / 4
colors = {
        "RigidStatic": (255, 255, 255, 255),
        "RigidDynamic": (127, 127, 127, 255),
        "RigidActor": (127, 127, 230, 255)
}


def fix_vertices(vertices):
    """ 
    vertices:list
    从“以左下角为原点，y轴向下”的坐标系转为“屏幕中心的坐标系,y下”
    """
    viewZoom=3
    return [(int(SCREEN_OFFSETX + viewZoom*v[0]), int(SCREEN_OFFSETY - viewZoom*v[1])) for v in vertices]

def _draw_test(screen):
    v=([10,10],[10,20],[20,20],[20,10])
    pygame.draw.polygon(screen, (255,0,0,255), v, 1)

    v=([700,500],[710,500],[710,510],[700,510])
    pygame.draw.polygon(screen, (0,255,0,255), v, 1)

def _draw_box(box, screen, actor,flag):
        """
        绘制box
        """
        global_pose=actor.get_global_pose()
        offset_x=global_pose[0][0]
        offset_y=global_pose[0][1]

        vs=box.get_box_half_extents()
        origin_vertices=([PPM*(vs[0]+offset_x),PPM*(vs[1]+offset_y)],
                        [PPM*(vs[0]+offset_x),PPM*(-vs[1]+offset_y)],
                        [PPM*(-vs[0]+offset_x),PPM*(-vs[1]+offset_y)],
                        [PPM*(-vs[0]+offset_x),PPM*(vs[1]+offset_y)])
        vertices=fix_vertices(origin_vertices)

        # vertices=origin_vertices
        pygame.draw.polygon(
            screen, [c / 2.0 for c in colors[flag]], vertices, 0)  # width=0
        pygame.draw.polygon(screen, colors[flag], vertices, 1)  # width=1


def _draw_mesh(mesh, screen, actor,flag):
        global_pose=actor.get_global_pose()
        offset_x=global_pose[0][0]
        offset_y=global_pose[0][1]
        # print("draw mesh-global:")
        # print(offset_x)

        faces=mesh.get_shape_data()
        face_num=faces.shape[0]

        for i in range(face_num):
            # print(faces[i])
            v1=(PPM*(faces[i][0]+offset_x),PPM*(faces[i][1]+offset_y))
            v2=(PPM*(faces[i][3]+offset_x),PPM*(faces[i][4]+offset_y))
            v3=(PPM*(faces[i][6]+offset_x),PPM*(faces[i][7]+offset_y))

            origin_vertices=(v1,v2,v3)
            vertices=fix_vertices(origin_vertices)
            # vertices=origin_vertices
            # print(vertices)
            pygame.draw.polygon(screen, colors[flag], vertices, 1)


def draw_scene(screen, balancer):
        # Draw the world
        # print("draw---")
        scene=balancer.scene
        _draw_test(screen)
        actors = scene.get_static_rigid_actors()
        
        for one_actor in actors:
            shapes = one_actor.get_atached_shapes()
            # print("shape size:{}".format(len(shapes)))
            for one_shape in shapes:
                geo_type = one_shape.get_geometry_type()
                if geo_type == GeometryType.BOX:
                    _draw_box(one_shape, screen, one_actor,"RigidStatic")
                elif geo_type == GeometryType.TRIANGLEMESH:
                    _draw_mesh(one_shape, screen, one_actor,"RigidStatic")
                else:
                    pass
                    # print(geo_type)

        actors=scene.get_dynamic_rigid_actors()
        for one_actor in actors:
            shapes = one_actor.get_atached_shapes()
            # print("shape size:{}".format(len(shapes)))
            for one_shape in shapes:
                geo_type = one_shape.get_geometry_type()
                if geo_type == GeometryType.BOX:
                    _draw_box(one_shape, screen, one_actor,"RigidDynamic")
                elif geo_type == GeometryType.TRIANGLEMESH:
                    _draw_mesh(one_shape, screen, one_actor,"RigidDynamic")
                else:
                    pass
                    # print(geo_type)


class Keys(object):
    pass




class OldFramework(object):
    name = 'TH'
    description = ''
    view_balancer=None

    def __init__(self,balancer):
        self.view_balancer=balancer

        print('Initializing pygame framework...')
        # Pygame Initialization
        pygame.init()
        caption = "Python Box2D Testbed - OLD backend - " + self.name
        pygame.display.set_caption(caption)

        # Screen and debug draw
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.font = pygame.font.Font(None, 15)

        # self.groundbody = self.world.CreateBody()

    def run(self):
        """
        Main loop.

        Updates the world and then the screen.
        """

        start_time = time.time()
        running = True
        clock = pygame.time.Clock()
        rate = Rate(2400)
        for _ in range(1000000000):
            
            if running==False:
                break
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key == Keys.K_ESCAPE):
                    running = False
                elif event.type == KEYDOWN:
                    self.Keyboard(event.key)
                elif event.type == KEYUP:
                    self.KeyboardUp(event.key)
            
            self.screen.fill((0, 0, 0))
            self.textLine = 15

            # Step the world
            self.view_balancer.update(rate.period())
            if _ & 0xff == 0xff:
                print("simulate time per 256 frames:", time.time() - start_time)
                start_time = time.time()

            draw_scene(self.screen, self.view_balancer)

            # Draw the name of the test running
            self.Print(self.name, (127, 127, 255))

            if self.description:
                # Draw the name of the test running
                for s in self.description.split('\n'):
                    self.Print(s, (127, 255, 127))

            pygame.display.flip()
            clock.tick(TARGET_FPS)
            self.fps = clock.get_fps()

    def Print(self, str, color=(229, 153, 153, 255)):
        """
        Draw some text at the top status lines
        and advance to the next line.
        """
        self.screen.blit(self.font.render(
            str, True, color), (5, self.textLine))
        self.textLine += 15

    def Keyboard(self, key):
        """
        Callback indicating 'key' has been pressed down.
        The keys are mapped after pygame's style.

         from .framework import Keys
         if key == Keys.K_z:
             ...
        """
        pass

    def KeyboardUp(self, key):
        """
        Callback indicating 'key' has been released.
        See Keyboard() for key information
        """
        pass
