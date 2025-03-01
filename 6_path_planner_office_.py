import rospy
import heapq
from std_msgs.msg import String

# Load the map data
map_data = {
    "Hallway0": {
        "connections": {
            "HallwayA": {"direction": "west", "distance": 4},
            "HallwayJ": {"direction": "south", "distance": 6.5}
        }
    },
    "HallwayA": {
        "connections": {
            "Hallway0": {"direction": "east", "distance": 4},
            "HallwayB": {"direction": "west", "distance": 3.5},
            "RoomA": {"direction": "north", "distance": 0.5}
        }
    },
    "RoomA": {
        "connections": {
            "HallwayA": {"direction": "south", "distance": 0.5}
        }
    },
    "HallwayB": {
        "connections": {
            "HallwayA": {"direction": "east", "distance": 3.5},
            "HallwayC": {"direction": "west", "distance": 1.5},
            "RoomB": {"direction": "north", "distance": 0.5}
        }
    },
    "RoomB": {
        "connections": {
            "HallwayB": {"direction": "south", "distance": 0.5}
        }
    },
    "HallwayC": {
        "connections": {
            "HallwayB": {"direction": "east", "distance": 1.5},
            "HallwayD": {"direction": "west", "distance": 1.5},
            "RoomC": {"direction": "north", "distance": 0.5}
        }
    },
    "RoomC": {
        "connections": {
            "HallwayC": {"direction": "south", "distance": 0.5}
        }
    },
    "HallwayD": {
        "connections": {
            "HallwayC": {"direction": "east", "distance": 1.5},
            "HallwayE": {"direction": "south", "distance": 2},
            "RoomD": {"direction": "west", "distance": 0.5}
        }
    },
    "RoomD": {
        "connections": {
            "HallwayD": {"direction": "east", "distance": 0.5}
        }
    },
    "HallwayE": {
        "connections": {
            "HallwayD": {"direction": "north", "distance": 2},
            "HallwayF": {"direction": "south", "distance": 3.8},
            "RoomE": {"direction": "west", "distance": 0.5}
        }
    },
    "RoomE": {
        "connections": {
            "HallwayE": {"direction": "east", "distance": 0.5}
        }
    },
    "HallwayF": {
        "connections": {
            "HallwayE": {"direction": "north", "distance": 3.8},
            "HallwayG": {"direction": "south", "distance": 0.7},
            "RoomF": {"direction": "west", "distance": 0.5}
        }
    },
    "RoomF": {
        "connections": {
            "HallwayF": {"direction": "east", "distance": 0.5}
        }
    },
    "HallwayG": {
        "connections": {
            "HallwayF": {"direction": "north", "distance": 0.7},
            "HallwayH": {"direction": "east", "distance": 1},
            "HallwayG1": {"direction": "south", "distance": 1.5}
        }
    },
    "HallwayG1": {
        "connections": {
            "HallwayG": {"direction": "north", "distance": 1.5},
            "RoomG1": {"direction": "south", "distance": 0.1},
            "RoomH": {"direction": "east", "distance": 1},
            "RoomG2": {"direction": "east", "distance": 0.5}
        }
    },
    "RoomG1": {
        "connections": {
            "HallwayG1": {"direction": "north", "distance": 0.1}
        }
    },
    "RoomG2": {
        "connections": {
            "HallwayG1": {"direction": "west", "distance": 0.5},
            "HallwayH": {"direction": "west", "distance": 0.5}
        }
    },
    "RoomH": {
        "connections": {
            "HallwayG1": {"direction": "west", "distance": 1},
            "HallwayH": {"direction": "north", "distance": 1.5}
        }
    },
    "HallwayH": {
        "connections": {
            "HallwayG": {"direction": "west", "distance": 1},
            "HallwayI": {"direction": "east", "distance": 3.5}
        }
    },
    "HallwayI": {
        "connections": {
            "HallwayH": {"direction": "west", "distance": 3.5},
            "HallwayII": {"direction": "east", "distance": 2},
            "RoomI": {"direction": "north", "distance": 0.5}
        }
    },
    "HallwayII": {
        "connections": {
            "HallwayJ": {"direction": "east", "distance": 4},
            "HallwayI": {"direction": "west", "distance": 2},
            "RoomJ": {"direction": "north", "distance": 0.5}
        }
    },
    "RoomI": {
        "connections": {
            "HallwayI": {"direction": "south", "distance": 0.5}
        }
    },
    "RoomJ": {
        "connections": {
            "HallwayII": {"direction": "south", "distance": 0.5}
        }
    },
    "HallwayJ": {
        "connections": {
            "HallwayII": {"direction": "west", "distance": 4},
            "HallwayK": {"direction": "east", "distance": 3.5},
            "Hallway0": {"direction": "north", "distance": 6.5},
            "Noisy_area_2": {"direction": "south", "distance": 11}
        }
    },
    "HallwayK": {
        "connections": {
            "HallwayJ": {"direction": "west", "distance": 3.5},
            "RoomK": {"direction": "north", "distance": 0.5},
            "HallwayL": {"direction": "east", "distance": 1.5}
        }
    },
    "RoomK": {
        "connections": {
            "HallwayK": {"direction": "south", "distance": 0.5}
        }
    },
    "HallwayL": {
        "connections": {
            "HallwayK": {"direction": "west", "distance": 1.5},
            "RoomM": {"direction": "south", "distance": 0.5},
            "HallwayM": {"direction": "east", "distance": 4}
        }
    },
    "RoomM": {
        "connections": {
            "HallwayL": {"direction": "north", "distance": 0.5}
        }
    },
    "HallwayM": {
        "connections": {
            "HallwayL": {"direction": "west", "distance": 4},
            "RoomN": {"direction": "south", "distance": 0.5},
            "RoomO": {"direction": "north", "distance": 0.5},
            "HallwayN": {"direction": "east", "distance": 1.5}
        }
    },
    "RoomN": {
        "connections": {
            "HallwayM": {"direction": "north", "distance": 0.5}
        }
    },
    "RoomO": {
        "connections": {
            "HallwayM": {"direction": "south", "distance": 0.5}
        }
    },
    "HallwayN": {
        "connections": {
            "HallwayM": {"direction": "west", "distance": 1.5},
            "RoomP": {"direction": "north", "distance": 0.5},
            "HallwayO": {"direction": "east", "distance": 3}
        }
    },
    "RoomP": {
        "connections": {
            "HallwayN": {"direction": "south", "distance": 0.5}
        }
    },
    "HallwayO": {
        "connections": {
            "HallwayN": {"direction": "west", "distance": 3},
            "Womantoilet": {"direction": "south", "distance": 0.5},
            "HallwayP": {"direction": "east", "distance": 2}
        }
    },
    "Womantoilet": {
        "connections": {
            "HallwayO": {"direction": "south", "distance": 0.5}
        }
    },
    "RoomQ": {
        "connections": {
            "HallwayO": {"direction": "north", "distance": 0.5}
        }
    },
    "HallwayP": {
        "connections": {
            "HallwayQ": {"direction": "east", "distance": 2},
            "RoomR": {"direction": "north", "distance": 0.5},
            "HallwayO": {"direction": "west", "distance": 2}
        }
    },
    "RoomR": {
        "connections": {
            "HallwayP": {"direction": "south", "distance": 0.5}
        }
    },
    "HallwayQ": {
        "connections": {
            "HallwayP": {"direction": "west", "distance": 2},
            "HallwayR": {"direction": "south", "distance": 2}
        }
    },
    "HallwayR": {
        "connections": {
            "HallwayQ": {"direction": "north", "distance": 2},
            "Mantoilet": {"direction": "west", "distance": 0.5},
            "HallwayS": {"direction": "south", "distance": 4}
        }
    },
    "Mantoilet": {
        "connections": {
            "HallwayR": {"direction": "east", "distance": 0.5}
        }
    },
    "HallwayS": {
        "connections": {
            "HallwayR": {"direction": "north", "distance": 4},
            "HallwayT": {"direction": "south", "distance": 5},
            "Stairs": {"direction": "east", "distance": 0.5}
        }
    },
    "Stairs": {
        "connections": {
            "HallwayS": {"direction": "west", "distance": 0.5}
        }
    },
    "HallwayT": {
        "connections": {
            "HallwayS": {"direction": "north", "distance": 5},
            "HallwayU": {"direction": "west", "distance": 2}
        }
    },
    "HallwayU": {
        "connections": {
            "HallwayT": {"direction": "east", "distance": 2},
            "RoomU": {"direction": "north", "distance": 1},
            "HallwayV": {"direction": "west", "distance": 2}
        }
    },
    "RoomU": {
        "connections": {
            "HallwayU": {"direction": "south", "distance": 1}
        }
    },
    "HallwayV": {
        "connections": {
            "HallwayU": {"direction": "east", "distance": 5},
            "RoomV": {"direction": "north", "distance": 1},
            "HallwayW": {"direction": "west", "distance": 5}
        }
    },
    "RoomV": {
        "connections": {
            "HallwayV": {"direction": "south", "distance": 1}
        }
    },
    "HallwayW": {
        "connections": {
            "HallwayV": {"direction": "east", "distance": 5},
            "Elevator": {"direction": "north", "distance": 1},
            "HallwayX": {"direction": "west", "distance": 3.5}
        }
    },
    "Elevator": {
        "connections": {
            "HallwayW": {"direction": "south", "distance": 1}
        }
    },
    "HallwayX": {
        "connections": {
            "HallwayW": {"direction": "east", "distance": 3.5},
            "Noisy_area_2": {"direction": "west", "distance": 5}
        }
    },
    "Noisy_area_2": {
        "connections": {
            "HallwayJ": {"direction": "north", "distance": 11},
            "HallwayX": {"direction": "east", "distance": 5}
        }
    }
}

def dijkstra(map_data, start, goal):
    """
    Returns the shortest distance and path from start to goal using Dijkstra's algorithm.
    """
    queue = [(0, start, [])]
    visited = set()
    while queue:
        (current_distance, current_node, path) = heapq.heappop(queue)
        if current_node in visited:
            continue
        path = path + [current_node]
        if current_node == goal:
            return current_distance, path
        visited.add(current_node)
        for neighbor, info in map_data[current_node]["connections"].items():
            if neighbor not in visited:
                distance = info["distance"]
                heapq.heappush(queue, (current_distance + distance, neighbor, path))
    return float("inf"), []

def compute_turning_instruction(current_facing, new_facing):
    """
    Returns a turning instruction as a string.
    Instead of "Turn around", it returns "Turn right 180" for a 180° turn.
    """
    directions_order = ['north', 'east', 'south', 'west']
    idx_current = directions_order.index(current_facing)
    idx_new = directions_order.index(new_facing)
    diff = (idx_new - idx_current) % 4
    if diff == 0:
        return ""
    elif diff == 1:
        return "Turn right 90"
    elif diff == 2:
        return "Turn right 180"
    elif diff == 3:
        return "Turn left 90"

def construct_directions(path, map_data, starting_heading):
    """
    Constructs directions with location headings appended.
    For each segment, if the required movement direction differs from the current facing,
    a turning instruction is output (with the from location annotated both with the current facing and the new facing).
    Then, a movement instruction is printed using "go forward".
    """
    directions = []
    total_distance = 0
    current_facing = starting_heading.lower()

    for i in range(len(path) - 1):
        from_node = path[i]
        to_node = path[i + 1]
        info = map_data[from_node]["connections"][to_node]
        required_direction = info["direction"]
        distance = info["distance"]
        total_distance += distance

        # If a turn is needed at the current node, output a turn instruction.
        if required_direction != current_facing:
            turn_inst = compute_turning_instruction(current_facing, required_direction)
            # Turn instruction shows the from_node with the current facing changing to the required facing.
            directions.append(f"{turn_inst} From {from_node}_{current_facing} to reach {from_node}_{required_direction},")
            current_facing = required_direction  # update current facing

        # Output the movement instruction.
        directions.append(f"From {from_node}_{current_facing}, go forward {distance} to reach {to_node}_{current_facing},")
    
    return directions, total_distance

def route_calculation_callback(msg):
    """
    Callback for processing incoming messages.
    Expected input: "Location_heading destination" (e.g. "Hallway0_west RoomE")
    """
    rospy.loginfo(f"Received message: '{msg.data}'")
    data = msg.data.strip().split()
    if len(data) != 2:
        result = ("Invalid input format. Expected: 'Location_heading destination' (e.g. Hallway0_west RoomE)")
        rospy.logwarn(result)
        route_pub.publish(result)
        return

    start_token = data[0]
    destination = data[1]

    if "_" not in start_token:
        result = ("Invalid start token format. Expected format: 'Location_heading' (e.g. Hallway0_west)")
        rospy.logwarn(result)
        route_pub.publish(result)
        return

    start_location, starting_heading = start_token.split('_', 1)

    if start_location not in map_data or destination not in map_data:
        result = (f"Invalid start or destination.\nYou entered: start='{start_location}', destination='{destination}'.\nExample usage: 'Hallway0_west RoomE'")
        rospy.logwarn(result)
        route_pub.publish(result)
        return

    if starting_heading.lower() not in ['north', 'east', 'south', 'west']:
        result = "Invalid starting heading. Please use one of: north, east, south, west."
        rospy.logwarn(result)
        route_pub.publish(result)
        return

    distance, path = dijkstra(map_data, start_location, destination)
    if path:
        directions, total_distance = construct_directions(path, map_data, starting_heading)
        direction_message = "\n".join(directions)
        shortest_path_message = f"Shortest distance: {distance:.2f} units\nDirections:\n{direction_message}"
    else:
        shortest_path_message = f"No path found from {start_location} to {destination}"

    rospy.loginfo(f"Publishing result: {shortest_path_message}")
    route_pub.publish(shortest_path_message)

if __name__ == '__main__':
    rospy.init_node('route_calculation_node', anonymous=True)
    route_pub = rospy.Publisher('/route_calculation_subscriber', String, queue_size=10)
    rospy.Subscriber('/route_calculation_publisher', String, route_calculation_callback)
    rospy.spin()
