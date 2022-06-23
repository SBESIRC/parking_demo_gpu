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

from .park_render import ParkRender

TARGET_FPS = 60
# PPM = 2.0  # scale
SCREEN_WIDTH, SCREEN_HEIGHT = 1600, 900
TIMESTEP = 1.0 / TARGET_FPS
VEL_ITERS, POS_ITERS = 10, 10

try:
    from .pygame_gui import (fwGUI, gui)
    GUIEnabled = True
except Exception as ex:
    print('Unable to load PGU; menu disabled.')
    print('(%s) %s' % (ex.__class__.__name__, ex))
    GUIEnabled = False

class AnchorUserData(metaclass=UserDataMeta):
    pass

class Keys(object):
    pass


# The following import is only needed to do the initial loading and
# overwrite the Keys class.
# Set up the keys (needed as the normal framework abstracts them between
# backends)


class ParkFramework(object):
    name = 'TH_PARK'
    description = ''
    view_balancer = None

    def __init__(self, balancer):
        self.view_balancer = balancer
        self.settings = fwSettings

        self.mouseJoint = None
        aabb_material = px.Material(static_friction=1, dynamic_friction=1, restitution=0.1)
        self.aabb_shape = px.Shape.create_box((1,1,1), aabb_material, is_exclusive=False)   # 用于检测overlap
        self.aabb_shape.setup_simulation_filtering(0,0)
        self.aabb_actor = px.RigidDynamic() # 用于检测overlap
        self.aabb_actor.attach_shape(self.aabb_shape)
        
        aabb_data = AnchorUserData(self.aabb_actor, Id=0, descp="aabb")
        self.aabb_actor.set_user_data(aabb_data)
        # print(aabb_actor.get_user_data().Type)
        self.mouseActor = px.RigidDynamic() # 鼠标点
        mouse_data = AnchorUserData(self.mouseActor, Id=0, descp="mouse")
        self.mouseActor.set_user_data(mouse_data)
        # self.mouseActor.set_rigid_body_flag(px.RigidBodyFlag.KINEMATIC, True)
        self.view_balancer.scene.add_actor(self.mouseActor)
        lock_2d(self.mouseActor)

        self.selectedActor = None
        self.mouseClickCnt = 0  # 左键点击计数
        
        self.lMouseDown = False
        self.rMouseDown = False
        

        self._viewZoom = 2.0
        self._viewOffset = None
        self._viewCenter = (balancer.centroid[0], balancer.centroid[1])

        self.gui_app = None
        self.gui_table = None
        self.setup_keys()

        print('Initializing park framework...')
        # Pygame Initialization
        pygame.init()
        caption = "Park backend - " + self.name
        pygame.display.set_caption(caption)

        # Screen and debug draw
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.screenSize = (SCREEN_WIDTH, SCREEN_HEIGHT)
        self.font = pygame.font.Font(None, 15)
        self.render = ParkRender(
            SCREEN_WIDTH, SCREEN_HEIGHT, self.viewCenter, self.viewZoom, framework=self)

        if GUIEnabled:
            self.gui_app = gui.App()
            self.gui_table = fwGUI(self.settings)
            container = gui.Container(align=-1, valign=-1)
            container.add(self.gui_table, 0, 0)
            self.gui_app.init(container)

        if self._viewCenter is None:
            print("center none")
            self.viewCenter = (100, 100)
        print(self.render.SCREEN_OFFSETX)
        # self.groundbody = self.world.CreateBody()

    def setup_keys(self):
        keys = [s for s in dir(pygame.locals) if s.startswith('K_')]
        for key in keys:
            value = getattr(pygame.locals, key)
            setattr(Keys, key, value)

    def set_scene(self, scene):
        self.scene = scene

    def setCenter(self, value):
        """
        Updates the view offset based on the center of the screen.

        Tells the debug draw to update its values also.
        """
        self._viewCenter = value
        
        self.render.updateCenter(self.viewCenter)
        
        # print("set center")
        self._viewOffset=(self.render.SCREEN_OFFSETX,self.render.SCREEN_OFFSETY)

    def setZoom(self, zoom):
        self._viewZoom = zoom
        self.render.updatePPM(self.viewZoom)
        self._viewOffset=(self.render.SCREEN_OFFSETX,self.render.SCREEN_OFFSETY)

    viewZoom = property(lambda self: self._viewZoom, setZoom,
                        doc='Zoom factor for the display')
    viewCenter = property(lambda self: self._viewCenter, setCenter,
                          doc='Screen center in camera coordinates')
    viewOffset = property(lambda self: self._viewOffset,
                          doc='The offset of the top-left corner of the screen')

    def checkEvents(self):
        """
        Check for pygame events (mainly keyboard/mouse events).
        Passes the events onto the GUI also.
        """
        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == Keys.K_ESCAPE):
                return False
            elif event.type == KEYDOWN:
                self._Keyboard_Event(event.key, down=True)
            elif event.type == KEYUP:
                self._Keyboard_Event(event.key, down=False)
            elif event.type == MOUSEBUTTONDOWN:
                p = self.ConvertScreenToWorld(*event.pos)
                if event.button == 1:  # left
                    self.lMouseDown = True
                    mods = pygame.key.get_mods()
                    if mods & KMOD_LSHIFT:
                        self.ShiftMouseDown(p)
                    else:
                        self.MouseDown(p)
                elif event.button == 2:  # middle
                    pass
                elif event.button == 3:  # right
                    self.rMouseDown = True
                elif event.button == 4:
                    self.viewZoom *= 1.1
                elif event.button == 5:
                    self.viewZoom /= 1.1
            elif event.type == MOUSEBUTTONUP:
                p = self.ConvertScreenToWorld(*event.pos)
                self.render.mouse_p = None # 追踪一下鼠标点在哪儿，画出来，为None时不画
                if event.button == 3:  # right
                    self.rMouseDown = False
                else:
                    self.lMouseDown = False
                    self.MouseUp(p)
            elif event.type == MOUSEMOTION:
                p = self.ConvertScreenToWorld(*event.pos)

                self.MouseMove(p)
                
                if self.rMouseDown:
                    self.render.mouse_p = p # 追踪一下鼠标点在哪儿，画出来，为None时不画
                    self.viewCenter = (self.viewCenter[0]-event.rel[0] /
                                       5.0, self.viewCenter[1]+event.rel[1] / 5.0)
                    self.render.mouse_p_flag="RMouse"
                    
                if self.lMouseDown:
                    self.render.mouse_p_flag="LMouse"
                    self.render.mouse_p = p # 追踪一下鼠标点在哪儿，画出来，为None时不画

            if GUIEnabled:
                self.gui_app.event(event)  # Pass the event to the GUI

        return True

    def run(self):
        """
        Main loop.

        Updates the world and then the screen.
        """
        if GUIEnabled:
            self.gui_table.updateGUI(self.settings)
        running = True
        self.viewCenter = self.viewCenter

        clock = pygame.time.Clock()
        rate = Rate(2400)
        while running:
            running = self.checkEvents()
            self.screen.fill((0, 0, 0))

            # Check keys that should be checked every loop (not only on initial
            # keydown)
            self.CheckKeys()
            # rate.frequency=self.settings.hz # FIXME:频率
            # Run the simulation loop
            # self.SimulationLoop()
            if GUIEnabled:
                self.gui_table.updateSettings(self.settings)
            # update the scene
            if self.settings.pause:
                self.view_balancer.update(0)    # PAUSE
            else:
                self.view_balancer.update(rate.period())
            # print(self.settings.hz)
            # render！
            self.render.draw_scene(self.screen, self.view_balancer) 

            if GUIEnabled and self.settings.drawMenu:
                self.gui_app.paint(self.screen)

            pygame.display.flip()
            clock.tick(self.settings.hz)
            self.fps = clock.get_fps()

            if GUIEnabled:
                self.gui_table.updateGUI(self.settings)

        # self.world.contactListener = None
        # self.world.destructionListener = None
        # self.world.renderer = None

    def Print(self, str, color=(229, 153, 153, 255)):
        """
        Draw some text at the top status lines
        and advance to the next line.
        """
        self.screen.blit(self.font.render(
            str, True, color), (5, self.textLine))
        self.textLine += 15

    def _Keyboard_Event(self, key, down=True):
        """
        Internal keyboard event, don't override this.

        Checks for the initial keydown of the basic testbed keys. Passes the unused
        ones onto the test via the Keyboard() function.
        """
        if down:
            if key == Keys.K_z:       # Zoom in
                self.viewZoom = min(1.1 * self.viewZoom, 50.0)
            elif key == Keys.K_x:     # Zoom out
                self.viewZoom = max(0.9 * self.viewZoom, 0.02)
            elif key == Keys.K_F1:    # Toggle drawing the menu
                self.settings.drawMenu = not self.settings.drawMenu
            elif key == Keys.K_F2:    # Do a single step
                self.settings.singleStep = True
                if GUIEnabled:
                    self.gui_table.updateGUI(self.settings)
            else:              # Inform the test of the key press
                self.Keyboard(key)
        else:
            self.KeyboardUp(key)

    def CheckKeys(self):
        """
        Check the keys that are evaluated on every main loop iteration.
        I.e., they aren't just evaluated when first pressed down
        """

        pygame.event.pump()
        self.keys = keys = pygame.key.get_pressed()
        if keys[Keys.K_LEFT]:
            self.viewCenter = (self.viewCenter[0]+1, self.viewCenter[1])
        elif keys[Keys.K_RIGHT]:
            self.viewCenter = (self.viewCenter[0]-1, self.viewCenter[1])

        if keys[Keys.K_UP]:
            self.viewCenter = (self.viewCenter[0], self.viewCenter[1]-1)
        elif keys[Keys.K_DOWN]:
            self.viewCenter = (self.viewCenter[0], self.viewCenter[1]+1)

        if keys[Keys.K_HOME]:
            self.viewZoom = 1.0
            self.viewCenter = (200.0, 400.0)

    def ConvertScreenToWorld(self, x, y):

        assert self.viewOffset[0] is not None
        return ((x - self.viewOffset[0]) / self.viewZoom ,
                (- y + self.viewOffset[1]) / self.viewZoom)

    def Keyboard(self, key):
        """
        Callback indicating 'key' has been pressed down.
        The keys are mapped after pygame's style.

         from framework import Keys
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

    def MouseDown(self, p):
        """
        Indicates that there was a left click at point p (world coordinates)
        """
        if self.mouseJoint is not None:
            return

        # Create a mouse joint on the selected body (assuming it's dynamic)
        # Make a small box.

        loc = np.array([p[0],p[1], 0], dtype=np.float64)


        #joint = px.D6Joint(self.mouseActor, loc)
        #joint.set_motion(px.D6Axis.X, px.D6Motion.FREE)
        #joint.set_motion(px.D6Axis.Y, px.D6Motion.FREE)
        #joint.set_motion(px.D6Axis.SWING2, px.D6Motion.FREE)
        #joint.set_kinemic_projection(flag=True, tolerance=0.1)
        #self.view_balancer.scene.add_actor(aabb_actor)

        scene=self.view_balancer.scene
        actors = scene.get_dynamic_rigid_actors()
        
        i=0
        for actor in actors:
            if actor.get_user_data().Type=="Anchor" :
                print("this is aabb actor:{}".format(i))
                continue
            query=actor.overlaps(self.aabb_actor)
            if query:
                self.selectedActor=actor
                self.render.selectedIndex=i

                loc1, _ = actor.get_global_pose()
                loc2, _ = self.mouseActor.get_global_pose()
                axis = loc2 - loc1
                print(axis)
                axis_len = np.linalg.norm(axis, 2)
                u_axis = axis / axis_len

                matrix = np.array([
                    [u_axis[0], -u_axis[1], 0],
                    [u_axis[1], u_axis[0], 0],
                    [0, 0, 1]], dtype=np.float64)
                q = quaternion.from_rotation_matrix(matrix)
                self.mouseJoint=px.D6Joint(self.mouseActor, actor, ((0,0,0), q), ((0,0,0), q)) # local poses
                self.mouseJoint.set_motion(px.D6Axis.X, px.D6Motion.FREE)
                self.mouseJoint.set_motion(px.D6Axis.Y, px.D6Motion.FREE)
                self.mouseJoint.set_motion(px.D6Axis.Z, px.D6Motion.FREE)
                self.mouseJoint.set_motion(px.D6Axis.SWING1, px.D6Motion.FREE)
                self.mouseJoint.set_motion(px.D6Axis.SWING2, px.D6Motion.FREE)
                self.mouseJoint.set_motion(px.D6Axis.TWIST, px.D6Motion.FREE)
                self.mouseJoint.set_drive(px.D6Drive.X,1000000,0,10000000,True)
                self.mouseJoint.set_drive(px.D6Drive.X,1000000,0,10000000,True)
                self.mouseJoint.set_drive_position((0,0,0))
                print("find")
                print(i)
                break
            i+=1
        self.mouseClickCnt+=1




    def MouseUp(self, p):
        """
        Left mouse button up.
        """
        print("mouse up")
        if self.mouseJoint:
            print("删除mouseJoint")
            self.mouseJoint.release()
            self.mouseJoint = None
            self.render.selectedIndex =None

    def MouseMove(self, p):
        """
        Mouse moved to point p, in world coordinates.
        """
        
        loc = np.array([p[0],p[1], 0], dtype=np.float64)
        self.aabb_actor.set_global_pose(loc)
        self.mouseActor.set_global_pose(loc)
        if self.mouseJoint:
            self.mouseJoint.set_drive_position((0,0,0))
        # self.mouseActor.set_kinematic_target(loc)

