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
from ..settings import fwSettings

TARGET_FPS = 60
# PPM = 2.0  # scale
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 600
TIMESTEP = 1.0 / TARGET_FPS
VEL_ITERS, POS_ITERS = 10, 10

try:
    from .pygame_gui import (fwGUI, gui)
    GUIEnabled = True
except Exception as ex:
    print('Unable to load PGU; menu disabled.')
    print('(%s) %s' % (ex.__class__.__name__, ex))
    GUIEnabled = False

class ParkAABB():
    def __init__(self,lowerBound,upperBound):
        self.lowerBound=lowerBound
        self.upperBound=upperBound

class ParkRender(object):

    colors = {
        "RigidStatic": (255, 255, 255, 255),
        "RigidDynamic": (127, 127, 127, 255),
        "RigidActor": (127, 127, 230, 255),
        "LMouse": (255, 0, 0, 255),
        "RMouse": (0, 255, 0, 255)
    }

    def __init__(self, screen_width, screen_height, viewCenter, viewZoom):
        self.SCREEN_WIDTH = screen_width
        self.SCREEN_HEIGHT = screen_height
        self.viewCenter = viewCenter

        self.PPM = viewZoom
        self.mouse_p = None

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
        return [(int(self.SCREEN_OFFSETX + self.PPM*v[0]), int(self.SCREEN_OFFSETY - self.PPM*v[1])) for v in vertices]
        # return [(int(self.SCREEN_OFFSETX + self.PPM*(v[0]-self.viewCenter[0])), int(self.SCREEN_OFFSETY - self.PPM*(v[1]-self.viewCenter[1]))) for v in vertices]

    def _draw_test(self, screen):
        v = ([10, 10], [10, 20], [20, 20], [20, 10])
        pygame.draw.polygon(screen, (255, 0, 0, 255), v, 1)

        v = ([700, 500], [710, 500], [710, 510], [700, 510])
        pygame.draw.polygon(screen, (0, 255, 0, 255), v, 1)

    def _draw_mouse_motion(self, point, screen, flag="LMouse"):

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
            screen, [c / 2.0 for c in self.colors[flag]], vertices, 0)  # width=0
        pygame.draw.polygon(screen, self.colors[flag], vertices, 1)  # width=1

    def _draw_box(self, box, screen, actor, flag):
        """
        绘制box
        """
        global_pose = actor.get_global_pose()
        offset_x = global_pose[0][0]
        offset_y = global_pose[0][1]

        vs = box.get_box_half_extents()
        origin_vertices = ([vs[0]+offset_x, vs[1]+offset_y],
                           [vs[0]+offset_x, -vs[1]+offset_y],
                           [-vs[0]+offset_x, -vs[1]+offset_y],
                           [-vs[0]+offset_x, vs[1]+offset_y])
        vertices = self.fix_vertices(origin_vertices)

        # vertices=origin_vertices
        pygame.draw.polygon(
            screen, [c / 2.0 for c in self.colors[flag]], vertices, 0)  # width=0
        pygame.draw.polygon(screen, self.colors[flag], vertices, 1)  # width=1

    def _draw_mesh(self, mesh, screen, actor, flag):
        global_pose = actor.get_global_pose()
        offset_x = global_pose[0][0]
        offset_y = global_pose[0][1]
        # print("draw mesh-global:")
        # print(offset_x)

        faces = mesh.get_shape_data()
        face_num = faces.shape[0]

        for i in range(face_num):
            # print(faces[i])
            v1 = (faces[i][0]+offset_x, faces[i][1]+offset_y)
            v2 = (faces[i][3]+offset_x, faces[i][4]+offset_y)
            v3 = (faces[i][6]+offset_x, faces[i][7]+offset_y)

            origin_vertices = (v1, v2, v3)
            vertices = self.fix_vertices(origin_vertices)
            # vertices=origin_vertices
            # print(vertices)
            pygame.draw.polygon(screen, self.colors[flag], vertices, 1)

    def draw_scene(self, screen, balancer):
        # Draw the world

        scene = balancer.scene
        # self._draw_test(screen)
        self._draw_mouse_motion(self.mouse_p, screen, "RMouse")
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
        for one_actor in actors:
            shapes = one_actor.get_atached_shapes()
            # print("shape size:{}".format(len(shapes)))
            for one_shape in shapes:
                geo_type = one_shape.get_geometry_type()
                if geo_type == GeometryType.BOX:
                    self._draw_box(one_shape, screen,
                                   one_actor, "RigidDynamic")
                elif geo_type == GeometryType.TRIANGLEMESH:
                    self._draw_mesh(one_shape, screen,
                                    one_actor, "RigidDynamic")
                else:
                    pass
                    # print(geo_type)


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
            SCREEN_WIDTH, SCREEN_HEIGHT, self.viewCenter, self.viewZoom)

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
                if event.button == 3:  # right
                    self.rMouseDown = False
                    self.render.mouse_p = None # 追踪一下鼠标点在哪儿，画出来，为None时不画
                else:
                    self.MouseUp(p)
            elif event.type == MOUSEMOTION:
                p = self.ConvertScreenToWorld(*event.pos)

                self.MouseMove(p)

                if self.rMouseDown:
                    self.viewCenter = (self.viewCenter[0]+event.rel[0] /
                                       5.0, self.viewCenter[1]+event.rel[1] / 5.0)

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

            # Run the simulation loop
            # self.SimulationLoop()
            if GUIEnabled:
                self.gui_table.updateSettings(self.settings)
            if self.settings.pause:
                self.view_balancer.update(0)    # PAUSE
            else:
                self.view_balancer.update(rate.period())
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
            self.viewCenter = (self.viewCenter[0]-1, self.viewCenter[1])
        elif keys[Keys.K_RIGHT]:
            self.viewCenter = (self.viewCenter[0]+1, self.viewCenter[1])

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
        aabb = ParkAABB(lowerBound=(p[0]-0.001, p[1]-0.001),
                      upperBound= (p[0]+0.001, p[1]+0.001))

        # Query the world for overlapping shapes.
        # query = fwQueryCallback(p)
        # self.world.QueryAABB(query, aabb)

        # if query.fixture:
        #     body = query.fixture.body
        #     # A body was selected, create the mouse joint
        #     self.mouseJoint = self.world.CreateMouseJoint(
        #         bodyA=self.groundbody,
        #         bodyB=body,
        #         target=p,
        #         maxForce=1000.0 * body.mass)
        #     body.awake = True

    def MouseUp(self, p):
        """
        Left mouse button up.
        """
        if self.mouseJoint:
            self.world.DestroyJoint(self.mouseJoint)
            self.mouseJoint = None

    def MouseMove(self, p):
        """
        Mouse moved to point p, in world coordinates.
        """
        self.mouseWorld = p
        if self.mouseJoint:
            self.mouseJoint.target = p
