import math
import heapq
import time
import tracemalloc
import networkx as nx
import osmnx as ox

# Haversine function for calculating distance
def get_haversine(lon1, lat1, lon2, lat2):
    lon1 = math.radians(lon1)
    lat1 = math.radians(lat1)
    lon2 = math.radians(lon2)
    lat2 = math.radians(lat2)
    
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    
    # the haversine formula
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a)) 
    radius = 6371000 # earth radius in m
    
    return c * radius

# Dijkstra's algorithm 
def run_dijkstra(graph, start, target):
    distances = {start: 0}
    queue = [(0, start)]
    visited = set()
    nodes_checked = 0

    while len(queue) > 0:
        current_dist, current = heapq.heappop(queue)
        
        if current in visited:
            continue
            
        visited.add(current)
        nodes_checked += 1
        
        if current == target:
            return current_dist, nodes_checked

        for neighbor in graph.neighbors(current):
            # choose the shortest edge if there are multiple
            edges = graph.get_edge_data(current, neighbor)
            
            road_length = float('inf')
            for edge in edges.values():
                if 'length' in edge and edge['length'] < road_length:
                    road_length = edge['length']
            
            new_dist = current_dist + road_length

            # if theres a shorter path to neighbor 
            if neighbor not in distances or new_dist < distances[neighbor]:
                distances[neighbor] = new_dist
                heapq.heappush(queue, (new_dist, neighbor))

    return float('inf'), nodes_checked

# A* algorithm 
def run_astar(graph, start, target):
    distances = {start: 0}
    
    # Getting target coordinates for heuristic calculation
    target_x = graph.nodes[target]['x']
    target_y = graph.nodes[target]['y']
    
    start_x = graph.nodes[start]['x']
    start_y = graph.nodes[start]['y']
    start_heuristic = get_haversine(start_x, start_y, target_x, target_y)
    
    queue = [(start_heuristic, start)]
    visited = set()
    nodes_checked = 0

    while len(queue) > 0:
        current_priority, current = heapq.heappop(queue)
        
        if current in visited:
            continue
            
        visited.add(current)
        nodes_checked += 1

        if current == target:
            return distances[current], nodes_checked

        for neighbor in graph.neighbors(current):
            
            edges = graph.get_edge_data(current, neighbor)
            
            road_length = float('inf')
            for edge in edges.values():
                if 'length' in edge and edge['length'] < road_length:
                    road_length = edge['length']
            
            new_dist = distances[current] + road_length

            if neighbor not in distances or new_dist < distances[neighbor]:
                distances[neighbor] = new_dist
                
                # Calculate straight line from this neighbor to the hospital
                neighbor_x = graph.nodes[neighbor]['x']
                neighbor_y = graph.nodes[neighbor]['y']
                heuristic = get_haversine(neighbor_x, neighbor_y, target_x, target_y)
                
                priority = new_dist + heuristic
                heapq.heappush(queue, (priority, neighbor))

    return float('inf'), nodes_checked

if __name__ == "__main__":
    print("Downloading Heidelberg city map... Please wait.")
   
    G = ox.graph_from_place("Heidelberg, Germany", network_type="drive")  #  drive network for car routing

    # Hospital coordinates
    hospitals = [
        {"name": "Universitatsklinikum", "lat": 49.4176, "lon": 8.6636},
        {"name": "Krankenhaus Salem", "lat": 49.4048, "lon": 8.6987},
        {"name": "St. Josefskrankenhaus", "lat": 49.4039, "lon": 8.6883},
        {"name": "Thoraxklinik", "lat": 49.4158, "lon": 8.6700},
        {"name": "Psychiatrisches Zentrum", "lat": 49.3947, "lon": 8.6901},
    ]

    # Convert GPS coordinates nodes on the map
    hospital_nodes = []
    for h in hospitals:
        node_id = ox.nearest_nodes(G, h["lon"], h["lat"])
        hospital_nodes.append({"name": h["name"], "node_id": node_id})

    # The 5 Scenarios
    scenarios = [
        {"name": "Hauptbahnhof", "lat": 49.4033, "lon": 8.6750},
        {"name": "Altstadt", "lat": 49.4105, "lon": 8.7073},
        {"name": "Rohrbach", "lat": 49.3900, "lon": 8.6850},
        {"name": "Neuenheim", "lat": 49.4230, "lon": 8.6850},
        {"name": "Kirchheim", "lat": 49.3980, "lon": 8.7200},
    ]

    print("\nStarting Tests...")
    scenario_num = 1
    for s in scenarios:
        start_node = ox.nearest_nodes(G, s["lon"], s["lat"])
        print(f"\n--- Scenario {scenario_num}: {s['name']} ---")
        scenario_num += 1

        for algo in ["Dijkstra", "A*"]:
            shortest_dist = float('inf')
            closest_hosp = ""
            total_explored = 0
            
            # Start tracking memory and time
            tracemalloc.clear_traces()
            tracemalloc.start()
            start_t = time.perf_counter()

            # Run algorithm for every hospital
            for h in hospital_nodes:
                if algo == "Dijkstra":
                    dist, explored = run_dijkstra(G, start_node, h["node_id"])
                else:
                    dist, explored = run_astar(G, start_node, h["node_id"])
                
                total_explored += explored
                
                # if we found a closer hospital, update the results
                if dist < shortest_dist:
                    shortest_dist = dist
                    closest_hosp = h["name"]

            # Stop tracking
            end_t = time.perf_counter()
            _, peak_mem = tracemalloc.get_traced_memory()
            tracemalloc.stop()

            # networkx for validation
            target_node_id = None
            for h in hospital_nodes:
                if h["name"] == closest_hosp:
                    target_node_id = h["node_id"]
                    break
                    
            try:
                if target_node_id is not None:
                    nx_dist = nx.shortest_path_length(G, start_node, target_node_id, weight="length")
                else:
                    nx_dist = float('inf')
            except nx.NetworkXNoPath:
                nx_dist = float('inf')

            print(f"- {algo} Algorithm -")
            print(f"Nearest Hospital: {closest_hosp}")
            print(f"Calculated Distance: {shortest_dist / 1000:.2f} km")
            print(f"Validation Distance (NetworkX): {nx_dist / 1000:.2f} km")
            print(f"Nodes Checked: {total_explored} nodes")
            print(f"Runtime: {end_t - start_t:.4f} seconds")
            print(f"Peak Memory usage: {peak_mem / 1024:.2f} KB")
