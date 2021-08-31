#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    ` or N       : next sensor
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    C            : change weather (Shift+C reverse)

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import carla

from examples.manual_control import (World, HUD, KeyboardControl, CameraManager,
                                     CollisionSensor, LaneInvasionSensor, GnssSensor, IMUSensor)
from client_bounding_boxes import ClientSideBoundingBoxes
from srunner.datamanager.collision_detect import *
import os
import argparse
import logging
import time
import pygame
import numpy as np
import random


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================

class WorldSR(World):
    restarted = False
    # 单位 m/s
    ego_speed = 15

    def __init__(self, carla_world, hud, args):
        self.args = args
        super(WorldSR, self).__init__(carla_world, hud, args)
        self._collision_algor = CollisionICWDetector()

    def restart(self):

        if self.restarted:
            return
        self.restarted = True

        if not self.args.scenario and not self.args.testcase1 and not self.args.testcase2:
            return

        if self.args.testcase1:
            self.build_test_case1()
        elif self.args.testcase2:
            self.build_test_case2()

        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713

        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0

        self._surroundingcars = []
        # Get the ego vehicle
        while self.player is None:
            print("Waiting for the ego vehicle...")
            time.sleep(1)
            possible_vehicles = self.world.get_actors().filter('vehicle.*')
            for vehicle in possible_vehicles:
                if vehicle.attributes['role_name'] == "hero":
                    print("Ego vehicle found")
                    self.player = vehicle
                else:
                    self._surroundingcars.append(vehicle)

        self.player_name = self.player.type_id
        if not self.args.waitstart:
            self.init_ego_velocity()
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)

        calibration = np.identity(3)
        calibration[0, 2] = self.args.width / 2.0
        calibration[1, 2] = self.args.height / 2.0
        calibration[0, 0] = calibration[1, 1] = self.args.width / (2.0 * np.tan(90 * np.pi / 360.0))
        self.camera_manager.sensor.calibration = calibration

        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def build_test_case1(self):
        self.spawn_ego_car()

    def build_test_case2(self):
        pass

    def spawn_ego_car(self):
        spawn_point = carla.Transform(carla.Location(x=-74.32, y=-50, z=0.5), carla.Rotation(yaw=90))
        blueprint = self.world.get_blueprint_library().filter('vehicle.tesla.model3')
        self.player = self.world.try_spawn_actor(blueprint, spawn_point)

    def tick(self, clock):
        if len(self.world.get_actors().filter(self.player_name)) < 1:
            return False
        self._collision_algor.detect(self.player, self._surroundingcars[0], self.hud, self.map)
        self.hud.tick(self, clock)
        return True

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)
        vehicles = self.world.get_actors().filter('vehicle.*')
        bounding_boxes = ClientSideBoundingBoxes.get_bounding_boxes(vehicles, self.camera_manager.sensor)
        ClientSideBoundingBoxes.draw_bounding_boxes(display, bounding_boxes)

    def init_ego_velocity(self):
        forward_vec = self.player.get_transform().get_forward_vector()
        velocity_vec = self.ego_speed * forward_vec
        self.player.set_target_velocity(velocity_vec)
        print("ego_velocity = {}".format(velocity_vec))
        # self.player.enable_constant_velocity(carla.Vector3D(16, 0, 0))


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================

def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        world = WorldSR(client.get_world(), hud, args)
        controller = KeyboardControl(world, args.autopilot)

        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)
            if controller.parse_events(client, world, clock):
                return
            if not world.tick(clock):
                return
            world.render(display)
            pygame.display.flip()

    finally:

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--scenario',
        default=False,
        type=bool,
        help='should load scenario first, and will run the scenario'
    )
    argparser.add_argument(
        '--testcase1',
        default=False,
        type=bool,
        help='run simple test case 1'
    )
    argparser.add_argument(
        '--testcase2',
        default=False,
        type=bool,
        help='run simple test case 2'
    )
    argparser.add_argument(
        '--waitstart',
        default=False,
        type=bool,
        help='should wait start to activate scenario'
    )
    args = argparser.parse_args()

    args.rolename = 'hero'  # Needed for CARLA version
    args.filter = "vehicle.*"  # Needed for CARLA version
    args.gamma = 2.2  # Needed for CARLA version
    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:
        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except Exception as error:
        logging.exception(error)


if __name__ == '__main__':
    main()