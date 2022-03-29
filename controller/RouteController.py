from abc import ABC, abstractmethod
import random
import os
import sys
from core.Util import *
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("No environment variable SUMO_HOME!")
import traci
import sumolib
import math
import copy
import numpy as np
from core.Util import ConnectionInfo, Vehicle

STRAIGHT = "s"
TURN_AROUND = "t"
LEFT = "l"
RIGHT = "r"
SLIGHT_LEFT = "L"
SLIGHT_RIGHT = "R"


class RouteController(ABC):
    """
    Base class for routing policy

    To implement a scheduling algorithm, implement the make_decisions() method.
    Please use the boilerplate code from the example, and implement your algorithm between
    the 'Your algo...' comments.

    make_decisions takes in a list of vehicles and network information (connection_info).
        Using this data, it should return a dictionary of {vehicle_id: decision}, where "decision"
        is one of the directions defined by SUMO (see constants above). Any scheduling algorithm
        may be injected into the simulation, as long as it is wrapped by the RouteController class
        and implements the make_decisions method.

    :param connection_info: object containing network information, including:
                            - out_going_edges_dict {edge_id: {direction: out_edge}}
                            - edge_length_dict {edge_id: edge_length}
                            - edge_index_dict {edge_index_dict} keep track of edge ids by an index
                            - edge_vehicle_count {edge_id: number of vehicles at edge}
                            - edge_list [edge_id]

    """

    def __init__(self, connection_info: ConnectionInfo):
        self.connection_info = connection_info
        print("IM IN THE ALGO!!!!!")
        self.direction_choices = [STRAIGHT, TURN_AROUND,
                                  SLIGHT_RIGHT, RIGHT, SLIGHT_LEFT, LEFT]

    def compute_local_target(self, decision_list, vehicle):
        current_target_edge = vehicle.current_edge
        try:
            path_length = 0
            i = 0

            # the while is used to make sure the vehicle will not assume it arrives the destination beacuse the target edge is too short.
            while path_length <= max(vehicle.current_speed, 20):
                if current_target_edge == vehicle.destination:
                    break
                if i >= len(decision_list):
                    raise UserWarning(
                        "Not enough decisions provided to compute valid local target. TRACI will remove vehicle."
                    )

                choice = decision_list[i]
                if choice not in self.connection_info.outgoing_edges_dict[current_target_edge]:
                    raise UserWarning(
                        "Invalid direction. TRACI will remove vehicle."
                    )
                current_target_edge = self.connection_info.outgoing_edges_dict[
                    current_target_edge][choice]
                path_length += self.connection_info.edge_length_dict[current_target_edge]

                if i > 0:
                    if decision_list[i - 1] == decision_list[i] and decision_list[i] == 't':
                        # stuck in a turnaround loop, let TRACI remove vehicle
                        return current_target_edge

                i += 1

        except UserWarning as warning:
            print(warning)

        return current_target_edge

    @abstractmethod
    def make_decisions(self, vehicles, connection_info):
        pass


class RandomPolicy(RouteController):
    """
    Example class for a custom scheduling algorithm.
    Utilizes a random decision policy until vehicle destination is within reach,
    then targets the vehicle destination.
    """

    def __init__(self, connection_info):
        super().__init__(connection_info)

    # def a_Star(self, start_edge, destination):

    def make_decisions(self, vehicles, connection_info):
        """
        A custom scheduling algorithm can be written in between the 'Your algo...' comments.
        -For each car in the vehicle batch, your algorithm should provide a list of future decisions.
        -Sometimes short paths result in the vehicle reaching its local TRACI destination before reaching its
         true global destination. In order to counteract this, ask for a list of decisions rather than just one.
        -This list of decisions is sent to a function that returns the 'closest viable target' edge
          reachable by the decisions - it is not the case that all decisions will always be consumed.
          As soon as there is enough distance between the current edge and the target edge, the compute_target_edge
          function will return.
        -The 'closest viable edge' is a local target that is used by TRACI to control vehicles
        -The closest viable edge should always be far enough away to ensure that the vehicle is not removed
          from the simulation by TRACI before the vehicle reaches its true destination

        :param vehicles: list of vehicles to make routing decisions for
        :param connection_info: object containing network information
        :return: local_targets: {vehicle_id, target_edge}, where target_edge is a local target to send to TRACI
        """
        dec_list = []
        vehicle = []
        local_targets = {}
        for vehicle in vehicles:
            start_edge = vehicle.current_edge
            
            '''
            Your algo starts here
            '''

            print("Using A*")
            
            beg = set([start_edge])
            fin = set([])

            cur = {}
            cur[start_edge] = 0

            cir = {}
            cir[start_edge] = start_edge

            while sp(beg) > 0:
                sum = None

                for x in beg:
                    if sum == None or cur[x] + self.z(x) < cur[sum] + self.z(sum):
                        sum = x

                if sum == None:
                    print('Path unavalible')
                    return None

                if sum == destination:
                    mark = []

                    while cir[sum] != sum:
                        mark.append(sum)
                        sum = cir[sum]

                        mark.append(start_edge)

                        mark.reverse()

                        print('Path display: {}'.format(mark))
                        return mark
                for (y, mass) in self.area(sum):

                    if y not in beg and y not in fin:
                        beg.add(y)
                        cir[y] = sum
                        cur[y] = cur[sum] + mass

                    else:
                        if cur[y] > cur[sum] + mass:
                            cur[y] = cur[sum] + mass
                            cir[y] = sum

                            if y in fin:
                                fin.remove(y)
                                beg.add(y)

                beg.remove(sum)
                fin.add(sum)

            print('Path unavalible')

            print("IM IN THE ALGO!!!!!!")
            #dec_list = []

            # Print out connection info
            print(connection_info.outgoing_edges_dict)
            # print("Im going left!")
            i = 0
            while i < 10:  # choose the number of decisions to make in advanced; depends on the algorithm and network
                # 6 choices available in total
                choice = self.direction_choices[random.randint(0, 5)]
                # Hardcode them to go left
                # choice = 'l'
                # dead end
                if len(self.connection_info.outgoing_edges_dict[start_edge].keys()) == 0:
                    break

                # make sure to check if it's a valid edge
                if choice in self.connection_info.outgoing_edges_dict[start_edge].keys():
                    dec_list.append(choice)
                    start_edge = self.connection_info.outgoing_edges_dict[start_edge][choice]

                    if i > 0:
                        if dec_list[i-1] == dec_list[i] and dec_list[i] == 't':
                            # stuck in a turnaround loop, let TRACI remove vehicle
                            break

                    i += 1

        
        #Your algo ends here
        
        local_targets[vehicle.vehicle_id] = self.compute_local_target(dec_list, vehicle)

        return local_targets

class aStarMod(RouteController):
    """
    Example class for a custom scheduling algorithm.
    Utilizes a random decision policy until vehicle destination is within reach,
    then targets the vehicle destination.
    """

    def __init__(self, connection_info):
        super().__init__(connection_info)

    def make_decisions(self, vehicles, connection_info):
        """
        A custom scheduling algorithm can be written in between the 'Your algo...' comments.
        -For each car in the vehicle batch, your algorithm should provide a list of future decisions.
        -Sometimes short paths result in the vehicle reaching its local TRACI destination before reaching its
         true global destination. In order to counteract this, ask for a list of decisions rather than just one.
        -This list of decisions is sent to a function that returns the 'closest viable target' edge
          reachable by the decisions - it is not the case that all decisions will always be consumed.
          As soon as there is enough distance between the current edge and the target edge, the compute_target_edge
          function will return.
        -The 'closest viable edge' is a local target that is used by TRACI to control vehicles
        -The closest viable edge should always be far enough away to ensure that the vehicle is not removed
          from the simulation by TRACI before the vehicle reaches its true destination

        :param vehicles: list of vehicles to make routing decisions for
        :param connection_info: object containing network information
        :return: local_targets: {vehicle_id, target_edge}, where target_edge is a local target to send to TRACI
        """
        # dec_list = []
        # vehicle = []
        local_targets = {}
        # print("Using A* method...")
        for vehicle in vehicles:
            start_edge = vehicle.current_edge
            print(start_edge)
            
            '''
            Your algo starts here
            '''
            #print("{}: current - {}, destination - {}".format(vehicle.vehicle_id, vehicle.current_edge, vehicle.destination))
            decision_list = []
            unvisited = {edge: 1000000000 for edge in self.connection_info.edge_list} # map of unvisited edges
            visited = {} # map of visited edges
            current_edge = vehicle.current_edge

            current_distance = self.connection_info.edge_length_dict[current_edge]
            unvisited[current_edge] = current_distance
            path_lists = {edge: [] for edge in self.connection_info.edge_list} #stores shortest path to each edge using directions
            while True:
                if current_edge not in self.connection_info.outgoing_edges_dict.keys():
                    continue
                for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[current_edge].items():
                    if outgoing_edge not in unvisited:
                        continue
                    edge_length = self.connection_info.edge_length_dict[outgoing_edge]
                    new_distance = current_distance + edge_length
                    # Heuristic should go here
                    if new_distance < unvisited[outgoing_edge]:
                        unvisited[outgoing_edge] = new_distance
                        current_path = copy.deepcopy(path_lists[current_edge])
                        current_path.append(direction)
                        path_lists[outgoing_edge] = copy.deepcopy(current_path)
                        # print("{} + {} : {} + {}".format(path_lists[current_edge], direction, path_edge_lists[current_edge], outgoing_edge))

                visited[current_edge] = current_distance
                del unvisited[current_edge]
                if not unvisited:
                    break
                if current_edge==vehicle.destination:
                    break
                possible_edges = [edge for edge in unvisited.items() if edge[1]]
                current_edge, current_distance = sorted(possible_edges, key=lambda x: x[1])[0]
                #print('{}:{}------------'.format(current_edge, current_distance))
            #current_edge = vehicle.current_edge


            for direction in path_lists[vehicle.destination]:
                decision_list.append(direction)

            local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)
        return local_targets

