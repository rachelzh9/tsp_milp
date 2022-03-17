"""Simple Vehicles Routing Problem (VRP).

   This is a sample using the routing library python wrapper to solve a VRP
   problem.
   A description of the problem can be found here:
   http://en.wikipedia.org/wiki/Vehicle_routing_problem.

   Distances are in meters.
"""
import random
import numpy as np
import matplotlib.pyplot as plt
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def distance(c1, c2):
    diff = (c1[0] - c2[0], c1[1] - c2[1])
    return np.sqrt(diff[0] * diff[0] + diff[1] * diff[1])


def createGraph(n, env_size):
    nodes = []
    adj = np.zeros((n, n))
    for i in range(n):
        nodes.append([random.uniform(0,1)*env_size, random.uniform(0,1)*env_size])
    nodes = np.array(nodes)
    for i in range(n):
        for j in range(i+1,n):
            adj[i, j] = distance(nodes[i,:], nodes[j,:])
            adj[j, i] = adj[i,j]

    return nodes, adj

def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = adj
    data['num_vehicles'] = 4
    data['depot'] = 0
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f'Objective: {solution.ObjectiveValue()}')
    max_route_distance = 0
    tours = []
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        tours.append([])
        while not routing.IsEnd(index):
            plan_output += ' {} -> '.format(manager.IndexToNode(index))
            tours[-1].append(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print('Maximum of the route distances: {}m'.format(max_route_distance))
    return tours


def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        3000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    tours = []
    if solution:
        tours = print_solution(data, manager, routing, solution)
    else:
        print('No solution found !')
    return tours

if __name__ == '__main__':
    nodes, adj = createGraph(50, 100)
    tours = main()
    plt.plot(nodes[:, 0], nodes[:, 1], 'ko', zorder=100)
    for i in range(4):
        plot_tour = []
        tour = tours[i]
        for c in tours[i]:
            node = nodes[int(c)]
            plot_tour.append(node)
        plot_tour = np.array(plot_tour)
        plt.plot(plot_tour[:, 0], plot_tour[:, 1])
    plt.show()