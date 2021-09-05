#Import libraries
from geopy.distance import great_circle #pip install geopy
from ortools.linear_solver import pywraplp #pip install ortools==9.0.9048 
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp



def create_data_model(gps_coordt):
    """ Store the data for the problem """
    data = {}
    #Locations
    data['distance_matrix'] = gps_coordt
    data['num_vehicles'] = 1
    data['start'] = 0 #Start location
    return data


def dist_between_coords(point_1, point_2):
    """ Calculate the distance between coordenates """
    """ https://en.wikipedia.org/wiki/Great-circle_distance """
    if point_1 == point_2:
        return 0
    else:    
        return(great_circle(point_1, point_2).miles)

def compute_distance_matrix(locations):
    distance_matrix = []
    row = []
    for i in range(len(locations)):
        for j in range(len(locations)):
            row.append(int((dist_between_coords(locations[i], locations[j]) * 100))) #Scaling the distance matrix
        distance_matrix.append(row)
        row = []         
    return distance_matrix  


def print_solution(manager, routing, solution):
    """Prints solution on console."""
    print('Objective: {} miles'.format(solution.ObjectiveValue()))
    index = routing.Start(0)
    plan_output = 'Route for vehicle 0:\n'
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    print(plan_output)
    plan_output += 'Route distance: {}miles\n'.format(route_distance)

        


def main():
    #Ponts (lat, lon)
    point_1 = [5.8142, -55.1762] 
    point_2 = [5.8551, -55.1391]
    point_3 = [5.8367, -55.1470]
    point_4 = [5.8184, -55.1713]
    point_5 = [5.8397, -55.2166]
    point_6 = [5.8530, -55.1345]
    point_7 = [5.8341, -55.1479]
    point_8 = [5.8272, -55.1847]
    point_9 = [5.8404, -55.1507]

    coords = []
    coords.append(point_1)
    coords.append(point_2)
    coords.append(point_3)
    coords.append(point_4)
    coords.append(point_5)
    coords.append(point_6)
    coords.append(point_7)
    coords.append(point_8)
    coords.append(point_9)

    distance_matrix = compute_distance_matrix(coords)

    # Instantiate the data problem.
    data = create_data_model(distance_matrix) 

    print(data['distance_matrix'])

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                        data['num_vehicles'], data['start'])
    # Create Routing Model.                                    
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(manager, routing, solution)



if __name__ == '__main__':
    main()    
