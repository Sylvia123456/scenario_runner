#!/usr/bin/env python

#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Non-signalized junctions: crossing negotiation:

The hero vehicle is passing through a junction without traffic lights
And encounters another vehicle passing across the junction.
"""

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      SyncArrival,
                                                                      KeepVelocity,
                                                                      StopVehicle,
                                                                      KeepAccelerationWithInitVelocity,
                                                                      SetSpeedWithAcc,
                                                                      DecelerateVelocity)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerRegion
from srunner.scenarios.basic_scenario import BasicScenario

class TestCollisionAlgorithm5(BasicScenario):
    """
    Implementation class for
    'Non-signalized junctions: crossing negotiation' scenario,
    (Traffic Scenario 10).

    This is a single ego vehicle scenario
    """

    # ego vehicle parameters
    _ego_vehicle_max_velocity = 20
    _ego_vehicle_driven_distance = 105

    # other vehicle
    _other_actor_max_brake = 1.0
    _other_actor_target_velocitys = [2.4, 2, 1.6, 2, 0.8, 3, 0.4]

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600):
        """
        Setup all relevant parameters and create scenario
        """

        self._other_actor_transform = None
        self._other_actor_transforms = []
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(TestCollisionAlgorithm5, self).__init__("TestCollisionAlgorithm5",
                                                      ego_vehicles,
                                                      config,
                                                      world,
                                                      debug_mode,
                                                      criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        # self._other_actor_transform = config.other_actors[0].transform
        # first_vehicle_transform = carla.Transform(
        #     carla.Location(config.other_actors[0].transform.location.x,
        #                    config.other_actors[0].transform.location.y,
        #                    config.other_actors[0].transform.location.z - 500),
        #     config.other_actors[0].transform.rotation)
        # first_vehicle = CarlaDataProvider.request_new_actor(config.other_actors[0].model, first_vehicle_transform)
        # first_vehicle.set_simulate_physics(enabled=False)
        # self.other_actors.append(first_vehicle)
        for actor in config.other_actors:
            self._other_actor_transforms.append(actor.transform)
            actor_transform = carla.Transform(
                carla.Location(actor.transform.location.x,
                               actor.transform.location.y,
                               actor.transform.location.z),
                actor.transform.rotation)
            vehicle = CarlaDataProvider.request_new_actor(actor.model, actor_transform, rolename=actor.rolename)
            vehicle.set_simulate_physics(enabled=False)
            self.other_actors.append(vehicle)

    def _create_behavior(self):
        """
        After invoking this scenario, it will wait for the user
        controlled vehicle to enter the start region,
        then make a traffic participant to accelerate
        until it is going fast enough to reach an intersection point.
        at the same time as the user controlled vehicle at the junction.
        Once the user controlled vehicle comes close to the junction,
        the traffic participant accelerates and passes through the junction.
        After 60 seconds, a timeout stops the scenario.
        """
        keep_velocity_others = []
        stop_other_triggers = []
        stop_other_pos = [
            # min_x, max_x, min_y, max_y
            [160, 170, -240, -250],
            [250, 260, -220, -210],
            [250, 260, -310, -300],
            [160, 170, -260, -240],
            [340, 345, -250, -240],
            [250, 265, -310, -300],
            [250, 260, -190, -180],
        ]

        # Creating leaf nodes
        start_other_trigger = InTriggerRegion(
            self.ego_vehicles[0],
            165, 167,
            -250, -240)

        sync_arrival = SyncArrival(
            self.other_actors[0], self.ego_vehicles[0],
            carla.Location(x=0, y=129))

        pass_through_trigger = InTriggerRegion(
            self.ego_vehicles[0],
            -20, 17,
            120, 140)

        for index, actor in enumerate(self.other_actors):
            if index == 1:
                # keep_velocity_with_acc = KeepAccelerationWithInitVelocity(actor, 2, 0.1)
                keep_velocity_with_acc = SetSpeedWithAcc(actor, 0.1, 2)
                keep_velocity_others.append(keep_velocity_with_acc)
            elif index == 3:
                decelerate_velocity = SetSpeedWithAcc(actor, -0.1, 2)
                keep_velocity_others.append(decelerate_velocity)
            else:
                keep_velocity = KeepVelocity(
                    actor, self._other_actor_target_velocitys[index])
                keep_velocity_others.append(keep_velocity)

        for index, actor in enumerate(self.other_actors):
            stop_other_trigger = InTriggerRegion(
                actor,
                stop_other_pos[index][0], stop_other_pos[index][1], stop_other_pos[index][2], stop_other_pos[index][3])
            stop_other_triggers.append(stop_other_trigger)

        # stop_other_trigger_0 = InTriggerRegion(
        #     self.other_actors[0],
        #     160, 170,
        #     -240, -250)
        #
        # stop_other_trigger_1 = InTriggerRegion(
        #     self.other_actors[1],
        #     250, 260,
        #     -220, -210)

        stop_other = StopVehicle(
            self.other_actors[0],
            self._other_actor_max_brake)

        end_condition = InTriggerRegion(
            self.ego_vehicles[0],
            300, 310,
            -250, -240
        )

        # Creating non-leaf nodes
        root = py_trees.composites.Sequence()
        scenario_sequence = py_trees.composites.Sequence()
        set_actors_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        sync_arrival_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity_other_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        # keep_velocity_0_parallel = py_trees.composites.Parallel(
        #     policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        # keep_velocity_1_parallel = py_trees.composites.Parallel(
        #     policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        for index, actor in enumerate(self.other_actors):
            set_actors_parallel.add_child(ActorTransformSetter(actor, self._other_actor_transforms[index]))

        # Building tree
        root.add_child(scenario_sequence)
        scenario_sequence.add_child(set_actors_parallel)
        scenario_sequence.add_child(start_other_trigger)
        scenario_sequence.add_child(keep_velocity_other_parallel)
        #  scenario_sequence.add_child(sync_arrival_parallel)
        scenario_sequence.add_child(stop_other)
        scenario_sequence.add_child(end_condition)
        for actor in self.other_actors:
            scenario_sequence.add_child(ActorDestroy(actor))

        # sync_arrival_parallel.add_child(sync_arrival)
        # sync_arrival_parallel.add_child(pass_through_trigger)

        for index, actor in enumerate(self.other_actors):
            keep_velocity_parallel = py_trees.composites.Parallel(
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
            )
            keep_velocity_parallel.add_child(keep_velocity_others[index])
            keep_velocity_parallel.add_child(stop_other_triggers[index])
            keep_velocity_other_parallel.add_child(keep_velocity_parallel)

        # keep_velocity_0_parallel.add_child(keep_velocity_other_0)
        # keep_velocity_0_parallel.add_child(stop_other_trigger_0)
        # keep_velocity_1_parallel.add_child(keep_velocity_other_1)
        # keep_velocity_1_parallel.add_child(stop_other_trigger_1)
        # keep_velocity_other_parallel.add_child(keep_velocity_0_parallel)
        # keep_velocity_other_parallel.add_child(keep_velocity_1_parallel)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collison_criteria = CollisionTest(self.ego_vehicles[0])
        criteria.append(collison_criteria)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
