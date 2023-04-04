import carla
import numpy as np
import networkx as nx
import logging
import matplotlib.pyplot as plt
from enum import Enum
from gym_carla.single_lane_env.carla_agent.global_route_planner import GlobalRoutePlanner
from gym_carla.single_lane_env.args import ROADS, STRAIGHT, CURVE, JUNCTION
from gym_carla.single_lane_env.tools.misc import test_waypoint


class CustomRoutePlanner(GlobalRoutePlanner):
    """
    class for generating chosen circuit's road topology,topology is saved with waypoints list
    vehicle always runs on the outer ring of chosen route
    """

    def __init__(self, wmap, sampling_resolution=1000.0) -> None:
        self._sampling_resolution = sampling_resolution
        self._wmap = wmap

        # code for simulation road generation
        self._route = []
        self._topology = []

        # generate circuit topology
        self._build_topology()
        # print(len(self._topology))
        # self._build_graph()
        # nx.draw(self._graph,with_labels=True,font_weight='bold')
        # plt.draw()
        # plt.show()

        # generate route waypoints list
        self._build_route()

    def get_route(self, ego_waypoint):
        return self._compute_next_waypoints(ego_waypoint, len(self._route))

    def get_spawn_points(self):
        """Vehicle can only be spawned on straight road, return transforms"""
        spawn_points = []
        for wp in self._route:
            if wp.road_id in STRAIGHT:
                temp = carla.Transform(wp.transform.location, wp.transform.rotation)
                # Increase the z value a little bit to avoid collison upon initializing
                temp.location.z += 0.1
                spawn_points.append(temp)

        return spawn_points

    def _build_route(self):
        begin = self._topology[0]
        self._route.append(begin['entry'])
        for wp in begin['path']:
            self._route.append(wp)
        # self._route.append(begin['exit'])
        indicator = begin['exit']
        iter = None
        for seg in self._topology:
            if seg['entry'].id == indicator.id:
                iter = seg
                break

        while indicator.id != begin['entry'].id:
            self._route.append(iter['entry'])
            for wp in iter['path']:
                self._route.append(wp)
            # self._route.append(iter['exit'])
            indicator = iter['exit']
            for seg in self._topology:
                if seg['entry'].id == indicator.id:
                    iter = seg
                    break

        # remove start
        # print(len(self._route))

    def _compute_next_waypoints(self, cur_wp, k=1):
        """
        Add new waypoints to the trajectory queue.

        param cur_wp: current waypoint
        param k: how many waypoints to compute
        :return: waypoint list
        """
        next_wps = []
        iter = None
        for i, wp in enumerate(self._route):
            if wp.id == cur_wp.id:
                iter = i
                break
            elif wp.transform.location.distance(cur_wp.transform.location) < self._sampling_resolution / 2:
                # can't find the exact waypoint, get an approximation
                iter = i
        if iter is None:
            logging.error("Current waypoint on route not found!")
        if iter + k < len(self._route):
            for i in range(k):
                next_wps.append(self._route[iter + i + 1])
        else:
            for i in range(len(self._route) - iter - 1):
                next_wps.append(self._route[iter + i + 1])
            for i in range(k - (len(self._route) - iter - 1)):
                next_wps.append(self._route[i])

        return next_wps

    def _build_topology(self):
        """
        This function retrieves topology from the server as a list of
        road segments as pairs of waypoint objects, and processes the
        topology into a list of dictionary objects with the following attributes

        - entry (carla.Waypoint): waypoint of entry point of road segment
        - entryxyz (tuple): (x,y,z) of entry point of road segment
        - exit (carla.Waypoint): waypoint of exit point of road segment
        - exitxyz (tuple): (x,y,z) of exit point of road segment
        - path (list of carla.Waypoint):  list of waypoints between entry to exit, separated by the resolution
        """
        # Retrieving waypoints to construct a detailed topology
        for segment in self._wmap.get_topology():
            wp1, wp2 = segment[0], segment[1]
            if test_waypoint(wp1) and test_waypoint(wp2):
                l1, l2 = wp1.transform.location, wp2.transform.location
                # Rounding off to avoid floating point imprecision
                x1, y1, z1, x2, y2, z2 = np.round([l1.x, l1.y, l1.z, l2.x, l2.y, l2.z], 0)
                wp1.transform.location, wp2.transform.location = l1, l2
                seg_dict = dict()
                seg_dict['entry'], seg_dict['exit'] = wp1, wp2
                seg_dict['entryxyz'], seg_dict['exitxyz'] = (x1, y1, z1), (x2, y2, z2)
                seg_dict['path'] = []
                endloc = wp2.transform.location
                if wp1.transform.location.distance(endloc) > self._sampling_resolution:
                    w = wp1.next(self._sampling_resolution)[0]
                    while w.transform.location.distance(endloc) > self._sampling_resolution:
                        if test_waypoint(w):
                            seg_dict['path'].append(w)
                        w = w.next(self._sampling_resolution)[0]
                    if test_waypoint(w):
                        seg_dict['path'].append(w)
                else:
                    next_wp = wp1.next(self._sampling_resolution)[0]
                    if test_waypoint(next_wp):
                        seg_dict['path'].append(next_wp)
                self._topology.append(seg_dict)

