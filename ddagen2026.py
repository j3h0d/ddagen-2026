import math
import networkx as nx

# 1. Pixel coordinates for p1 through p29
COORDS = {
    "p1": (17, 417),   "p2": (84, 442),   "p3": (114, 342),
    "p4": (67, 321),   "p5": (83, 285),   "p6": (229, 319),
    "p7": (236, 362),  "p8": (436, 362),  "p9": (416, 307),
    "p10": (258, 307), "p11": (278, 168), "p12": (269, 107),
    "p13": (223, 89),  "p14": (178, 111), "p15": (200, 142),
    "p16": (137, 131), "p17": (113, 228), "p18": (144, 238),
    "p19": (254, 260), "p20": (179, 255), "p21": (297, 44),
    "p22": (405, 43),  "p23": (408, 96),  "p24": (288, 92),
    "p25": (370, 239), "p26": (361, 172), "p27": (428, 258),
    "p28": (125, 299), "p29": (265, 237)
}

# 2. Main road topology
ADJACENCY = {
    "p1": ["p2"],
    "p2": ["p1", "p3"],
    "p3": ["p2", "p4", "p28"],
    "p4": ["p3", "p5"],
    "p5": ["p4", "p28"],
    "p28": ["p3", "p5", "p18", "p6"],
    "p6": ["p7", "p10", "p28"],
    "p7": ["p6", "p8"],
    "p8": ["p7", "p9"],
    "p9": ["p8", "p10", "p27"],
    "p27": ["p9", "p25"],
    "p25": ["p27", "p26", "p29"],
    "p26": ["p25", "p11"],
    "p11": ["p26", "p29", "p12"],
    "p12": ["p11", "p24", "p13"],
    "p14": ["p13", "p15", "p16"],
    "p16": ["p14", "p17"],
    "p17": ["p16", "p18"],
    "p18": ["p17", "p28", "p20"],
    "p20": ["p18", "p19", "p15"],
    "p15": ["p20", "p14"],
    "p10": ["p19", "p6", "p9"],
    "p24": ["p12", "p21", "p23"],
    "p23": ["p24", "p22"],
    "p22": ["p23", "p21"],
    "p21": ["p22", "p24"],
    "p13": ["p14", "p12"],
    "p19": ["p29", "p10", "p20"],
    "p29": ["p19", "p25", "p11"]
}

# 3. Company-to-booth mapping
COMPANIES = {
    "Ericsson": "p26",
    "Atlas Copco": "p11",
    "Nordea": "p10",
    "Revolut": "p25",
    "Svenska kärnkraft": "p16",
    "PWC": "p17",
    "Comsol": "p14",
    "Digpro solutions": "p21",
    "Flygresor.se": "p22",
    "Folksam": "p6",
    "Ida infront": "p7",
    "Lynx asset": "p21",
    "megger": "p14",
    "monitor erp": "p23",
    "Net insight": "p23",
    "treasury systems": "p11",
    "truesec": "p23"
}

# 4. Build graph with physical Euclidean edge weights
G = nx.Graph()
for u, neighbors in ADJACENCY.items():
    x1, y1 = COORDS[u]
    for v in neighbors:
        x2, y2 = COORDS[v]
        dist = math.hypot(x2 - x1, y2 - y1)
        G.add_edge(u, v, weight=dist)

# 5. Extract unique targets + start node
START_NODE = "p1"
unique_targets = sorted(list(set(COMPANIES.values())))
tour_nodes = [START_NODE] + [t for t in unique_targets if t != START_NODE]
n = len(tour_nodes)

# Precompute all-pairs shortest paths using Dijkstra
all_pairs_dist = dict(nx.all_pairs_dijkstra_path_length(G, weight="weight"))
all_pairs_path = dict(nx.all_pairs_dijkstra_path(G, weight="weight"))

cost_matrix = [[all_pairs_dist[tour_nodes[i]][tour_nodes[j]] for j in range(n)] for i in range(n)]

# 6. Held-Karp Dynamic Programming (Exact TSP)
memo = {}

def held_karp(mask, u):
    if mask == (1 << n) - 1:
        return cost_matrix[u][0], [0]
    state = (mask, u)
    if state in memo:
        return memo[state]
    
    best_dist = float("inf")
    best_path = []
    for v in range(n):
        if not (mask & (1 << v)):
            dist, path = held_karp(mask | (1 << v), v)
            total = cost_matrix[u][v] + dist
            if total < best_dist:
                best_dist = total
                best_path = [v] + path
                
    memo[state] = (best_dist, best_path)
    return best_dist, best_path

total_distance, optimal_idx_path = held_karp(1, 0)
node_order = [tour_nodes[i] for i in [0] + optimal_idx_path]

# 7. Print Output
print(f"Total Optimal Walking Distance: {total_distance:.1f} pixels\n")
print("Target Visiting Sequence:")
for node in node_order[1:-1]:
    visited_companies = [c for c, nd in COMPANIES.items() if nd == node]
    print(f"  Stop at {node:4s} -> {', '.join(visited_companies)}")

print("\nTurn-by-Turn Waypoint Navigation:")
full_path = []
for i in range(len(node_order) - 1):
    leg = all_pairs_path[node_order[i]][node_order[i+1]]
    if full_path:
        full_path.extend(leg[1:])
    else:
        full_path.extend(leg)

print(" -> ".join(full_path))
