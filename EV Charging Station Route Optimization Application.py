import json
import heapq


# Load the graph from the JSON file
def load_graph_from_json(filename):
    with open(filename, 'r') as file:
        graph = json.load(file)
    return graph


# Dijkstra's algorithm to find the shortest paths from the start node
def dijkstra(graph, start):
    # Priority queue initialized with the start node and its distance (0)
    queue = [(0, start)]
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    shortest_path = {node: [] for node in graph}

    while queue:
        # Pop the node with the smallest distance from the priority queue
        current_distance, current_node = heapq.heappop(queue)

        # Skip processing if we have already found a better path
        if current_distance > distances[current_node]:
            continue

        # Explore each neighbor of the current node
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight

            # Update distance and path if a shorter path is found
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                shortest_path[neighbor] = shortest_path[current_node] + [current_node]
                heapq.heappush(queue, (distance, neighbor))  # Add neighbor to the queue with updated distance

    return distances, shortest_path


# Recommend the best route based on the shortest distance to charging stations
def recommend_route(distances, charging_stations):
    min_distance = float('inf')
    best_station = None

    # Find the charging station with the minimum distance
    for station in charging_stations:
        if distances[station] < min_distance:
            min_distance = distances[station]
            best_station = station

    return best_station, min_distance


# Main function to execute the steps
def main():
    filename = 'network.json'

    try:
        # Load the graph from the JSON file
        graph = load_graph_from_json(filename)

        # Define the starting node and the charging stations
        start_node = 'A'
        charging_stations = ['H', 'K', 'Q', 'T']

        # Run Dijkstra's algorithm to find the shortest paths from the start node
        distances, shortest_path = dijkstra(graph, start_node)

        # Print the shortest paths and distances to each charging station
        for station in charging_stations:
            path = shortest_path[station] + [station]
            print(f"Shortest path to {station}: {path} with distance {distances[station]}")

        # Recommend the best charging station based on the shortest distance
        best_station, min_distance = recommend_route(distances, charging_stations)
        print(f"Recommended charging station: {best_station} with distance {min_distance}")

    except Exception as e:
        print(f"An error occurred: {e}")


# Execute the main function
if __name__ == "__main__":
    main()
