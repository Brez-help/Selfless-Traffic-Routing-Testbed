from controller.RouteController import RouteController
from core.Util import ConnectionInfo, Vehicle
from copy import deepcopy

class BellmanFordPolicy(RouteController):

    def __init__(self, connection_info):
        super().__init__(connection_info)

    def make_decisions(self, vehicles, connection_info):
        local_targets = {}

        for vehicle in vehicles:
            decision_list = []
            #no reason for the minimum value being set like so
            # just putting a high number like in the Dijkstra policy for its unvisited edges
            #trying to find the edge with the minimum distance from the current edge
            unvisited = {edge : 10000000 for edge in self.connection_info.edge_list}
            visited = {}

            current_edge = vehicle.current_edge
            path_lists = {edge: [] for edge in self.connection_info.edge_list}
            while True:
                if current_edge not in self.connection_info.outgoing_edges_dict.keys():
                    continue

                dist = {edge[1]: 10000000 for edge in self.connection_info.outgoing_edges_dict[current_edge].items()} # Distance from current edge
                dist[current_edge] = 0

                current_distance = self.connection_info.edge_length_dict[current_edge]
                unvisited[current_edge] = current_distance

                new_direction, new_outgoing = None, None
                min_weight = 10000000
                for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[current_edge].items():
                    if outgoing_edge not in unvisited:
                        continue

                    if dist[outgoing_edge] > dist[current_edge] + self.connection_info.edge_length_dict[outgoing_edge]:
                        dist[outgoing_edge] = dist[current_edge] + self.connection_info.edge_length_dict[outgoing_edge]

                    if min_weight > dist[outgoing_edge]:
                        min_weight = dist[outgoing_edge]
                        new_direction = direction
                        new_outgoing = outgoing_edge

                unvisited[new_outgoing] = min_weight
                current_path = deepcopy(path_lists[current_edge])
                current_path.append(new_direction)
                path_lists[new_outgoing] = deepcopy(current_path)

                visited[current_edge] = current_distance
                del unvisited[current_edge]

                if not unvisited or current_edge == vehicle.destination:
                    break

                current_edge, current_distance = new_outgoing, self.connection_info.edge_length_dict[new_outgoing]

            for direction in path_lists[vehicle.destination]:
                decision_list.append(direction)

            local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)

        return local_targets