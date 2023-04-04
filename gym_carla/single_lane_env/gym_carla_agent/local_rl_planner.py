""" This module contains a local rl planner to perform low-level waypoint following without PID controllers. """

from enum import Enum
from collections import deque
import random

import carla

from gym_carla.single_lane_env.carla_agent.local_planner import LocalPlanner, RoadOption
from gym_carla.single_lane_env.args import ROADS, STRAIGHT, CURVE, JUNCTION

from gym_carla.single_lane_env.tools.misc import get_lane_center, compute_magnitude_angle, \
    is_within_distance_ahead, test_waypoint


class LocalRlPlanner(LocalPlanner):
    """
    LocalPlanner implements the basic behavior of following a
    trajectory of waypoints that is generated on-the-fly.

    When multiple paths are available (intersections) this local planner makes a random choice,
    unless a given global plan has already been specified.
    """

    def __init__(self, vehicle, opt_dict={}):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param opt_dict: dictionary of arguments with different parameters:
            dt: time between simulation steps
            target_speed: desired cruise speed in Km/h
            sampling_radius: distance between the waypoints part of the plan
            lateral_control_dict: values of the lateral PID controller
            longitudinal_control_dict: values of the longitudinal PID controller
            max_throttle: maximum throttle applied to the vehicle
            max_brake: maximum brake applied to the vehicle
            max_steering: maximum steering applied to the vehicle
            offset: distance between the route waypoints and the center of the lane
        """
        self._vehicle = vehicle
        self._world = self._vehicle.get_world()
        self._map = self._world.get_map()

        self._vehicle_controller = None
        self.target_waypoint = None
        self.target_road_option = None

        self._waypoints_queue = deque(maxlen=10000)
        self._min_waypoint_queue_length = 100
        self._stop_waypoint_creation = False

        # add waypoint parameters
        self._sampling_radius = 4.0
        self._base_min_distance = 3.0

        self._target_waypoint = None
        self._buffer_size = 10

        self._waypoints_queue = deque(maxlen=600)
        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        self._target_road_option = RoadOption.LANEFOLLOW
        self._stop_waypoint_creation = False

        self._last_traffic_light = None
        self._proximity_threshold = 50

        self._waypoints_queue.append((self._current_waypoint, RoadOption.LANEFOLLOW))

        # Overload parameters
        if opt_dict:
            if 'base_min_distance' in opt_dict:
                self._base_min_distance = opt_dict['base_min_distance']
            if 'buffer_size' in opt_dict:
                self._buffer_size = opt_dict['buffer_size']
            if 'sampling_resolution' in opt_dict:
                self._sampling_radius = opt_dict['sampling_resolution']
            if 'vehicle_proximity' in opt_dict:
                self._proximity_threshold = opt_dict['vehicle_proximity']

        self._waypoint_buffer = deque(maxlen=self._buffer_size)

    def run_step(self):
        waypoints = self._get_waypoints()
        red_light, vehicle_front = self._get_hazard()
        return waypoints, red_light, vehicle_front

    def get_incoming_waypoint_and_direction(self, steps=3):
        """
        Returns direction and waypoint at a distance ahead defined by the user.

            :param steps: number of steps to get the incoming waypoint.
        """
        if len(self._waypoint_buffer) > steps:
            return self._waypoint_buffer[steps]
        else:
            try:
                wpt, direction = self._waypoint_buffer[-1]
                return wpt, direction
            except IndexError as i:
                return None, RoadOption.VOID

    def _get_waypoints(self):
        """
        Get the next waypoint list according to ego vehicle's current location
        based on _compute_next_waypoints
        Here we use the cut offed road
        """
        lane_center = get_lane_center(self._map, self._vehicle.get_location())
        _waypoints_queue = deque(maxlen=600)
        _waypoints_queue.append(lane_center)
        available_entries = _waypoints_queue.maxlen - len(self._waypoints_queue)
        k = min(available_entries, self._buffer_size)

        for _ in range(k):
            last_waypoint = _waypoints_queue[-1]
            next_waypoints = list(last_waypoint.next(self._sampling_radius))

            if len(next_waypoints) == 0:
                break
            elif len(next_waypoints) == 1:
                # only one option available ==> lanefollowing
                next_waypoint = next_waypoints[0]
            else:
                for i, wp in enumerate(next_waypoints):
                    if wp.road_id in ROADS:
                        next_waypoint = wp

            _waypoints_queue.append(next_waypoint)
        _waypoints_queue.popleft()
        return _waypoints_queue

    def _get_hazard(self):
        # retrieve relevant elements for safe navigation, i.e.: traffic lights
        # and other vehicles
        actor_list = self._world.get_actors()
        vehicle_list = actor_list.filter("*vehicle*")
        lights_list = actor_list.filter("*traffic_light*")

        # check possible obstacles
        vehicle = self._vehicle_hazard(vehicle_list)

        # check for the state of the traffic lights
        light_state = self._is_light_red_us_style(lights_list)

        return light_state, vehicle

    def _vehicle_hazard(self, vehicle_list):
        """
        Check if a given vehicle is an obstacle in our way. To this end we take
        into account the road and lane the target vehicle is on and run a
        geometry test to check if the target vehicle is under a certain distance
        in front of our ego vehicle.

        WARNING: This method is an approximation that could fail for very large
        vehicles, which center is actually on a different lane but their
        extension falls within the ego vehicle lane.

        :param vehicle_list: list of potential obstacle to check
        :return:
            - vehicle is the blocker object itself
        """

        ego_vehicle_location = self._vehicle.get_location()
        ego_vehicle_waypoint = self._map.get_waypoint(ego_vehicle_location)
        min_distance = self._proximity_threshold
        vehicle_front = None

        for target_vehicle in vehicle_list:
            # do not account for the ego vehicle
            if target_vehicle.id == self._vehicle.id:
                continue

            # if the object is not in our lane it's not an obstacle
            target_vehicle_waypoint = self._map.get_waypoint(target_vehicle.get_location())
            if not test_waypoint(target_vehicle_waypoint):
                continue

            loc = target_vehicle.get_location()
            if is_within_distance_ahead(loc, ego_vehicle_location,
                                        self._vehicle.get_transform().rotation.yaw,
                                        self._proximity_threshold):
                if ego_vehicle_location.distance(loc) < min_distance:
                    # Return the most close vehicle in front of ego vehicle
                    vehicle_front = target_vehicle
                    min_distance = ego_vehicle_location.distance(loc)

        return vehicle_front

    def _is_light_red_us_style(self, lights_list):
        """
        This method is specialized to check US style traffic lights.

        param lights_list: list containing TrafficLight objects
        :return: a tuple given by (bool_flag, traffic_light), where
            - bool_flag is True if there is a traffic light in RED
            affecting us and False otherwise
            - traffic_light is the object itself or None if there is no
            red traffic light affecting us
        """
        ego_vehicle_location = self._vehicle.get_location()
        ego_vehicle_waypoint = self._map.get_waypoint(ego_vehicle_location)

        if ego_vehicle_waypoint.is_intersection:
            # It is too late. Do not block the intersection! Keep going!
            return False

        if self._target_waypoint is not None:
            if self._target_waypoint.is_intersection:
                potential_lights = []
                min_angle = 180.0
                sel_magnitude = 0.0
                sel_traffic_light = None
                for traffic_light in lights_list:
                    loc = traffic_light.get_location()
                    magnitude, angle = compute_magnitude_angle(loc,
                                                               ego_vehicle_location,
                                                               self._vehicle.get_transform().rotation.yaw)
                    if magnitude < 80.0 and angle < min(25.0, min_angle):
                        sel_magnitude = magnitude
                        sel_traffic_light = traffic_light
                        min_angle = angle

                if sel_traffic_light is not None:
                    if self._last_traffic_light is None:
                        self._last_traffic_light = sel_traffic_light

                    if self._last_traffic_light.state == carla.libcarla.TrafficLightState.Red:
                        return True
                else:
                    self._last_traffic_light = None

        return False

