import math
from collections import defaultdict
import networkx as nx

# 1. Pixelkoordinater för p1 till p29
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

# 2. Huvudvägarnas topologi
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

# 3. Alla 17 företag kopplade till respektive nod
COMPANIES = [
    ("Ericsson", "p26"),
    ("Atlas Copco", "p11"),
    ("Nordea", "p10"),
    ("Revolut", "p25"),
    ("Svenska kärnkraft", "p16"),
    ("PwC", "p17"),
    ("Comsol", "p14"),
    ("Digpro solutions", "p21"),
    ("Flygresor.se", "p22"),
    ("Folksam", "p6"),
    ("Ida infront", "p7"),
    ("Lynx asset", "p21"),
    ("Megger", "p14"),
    ("Monitor ERP", "p23"),
    ("Net insight", "p23"),
    ("Treasury systems", "p11"),
    ("Truesec", "p23"),
]

# Gruppera företag per nod
node_to_companies = defaultdict(list)
for company, node in COMPANIES:
    node_to_companies[node].append(company)

# 4. Bygg graf med euklidiska avstånd som kantvikter
G = nx.Graph()
for u, neighbors in ADJACENCY.items():
    x1, y1 = COORDS[u]
    for v in neighbors:
        x2, y2 = COORDS[v]
        dist = math.hypot(x2 - x1, y2 - y1)
        G.add_edge(u, v, weight=dist)

START_NODE = "p1"
unique_targets = sorted(list(node_to_companies.keys()))
tour_nodes = [START_NODE] + [t for t in unique_targets if t != START_NODE]
n = len(tour_nodes)

# Beräkna kortaste vägen mellan alla målnoder via Dijkstra
all_pairs_dist = dict(nx.all_pairs_dijkstra_path_length(G, weight="weight"))
all_pairs_path = dict(nx.all_pairs_dijkstra_path(G, weight="weight"))

cost_matrix = [[all_pairs_dist[tour_nodes[i]][tour_nodes[j]] for j in range(n)] for i in range(n)]

# 5. Held-Karp dynamisk programmering (exakt TSP)
memo = {}

def solve_tsp(mask, u):
    if mask == (1 << n) - 1:
        return cost_matrix[u][0], [0]
    state = (mask, u)
    if state in memo:
        return memo[state]
    
    best_dist = float("inf")
    best_path = []
    for v in range(n):
        if not (mask & (1 << v)):
            dist, path = solve_tsp(mask | (1 << v), v)
            total = cost_matrix[u][v] + dist
            if total < best_dist:
                best_dist = total
                best_path = [v] + path
                
    memo[state] = (best_dist, best_path)
    return best_dist, best_path

total_dist, opt_indices = solve_tsp(1, 0)
optimal_node_order = [tour_nodes[i] for i in [0] + opt_indices]

# 6. Skriv ut besöksordning för samtliga 17 företag
print(f"Total optimal gångsträcka: {total_dist:.1f} pixlar\n")
print("Optimal besöksordning för alla 17 företag:")
stop_idx = 1
for node in optimal_node_order[1:-1]:
    for comp in node_to_companies[node]:
        print(f"  Stopp {stop_idx:2d}: {comp:<20} (Nod: {node})")
        stop_idx += 1

print("\nKomplett nod-för-nod-navigering:")
full_route = []
for i in range(len(optimal_node_order) - 1):
    leg = all_pairs_path[optimal_node_order[i]][optimal_node_order[i+1]]
    if full_route:
        full_route.extend(leg[1:])
    else:
        full_route.extend(leg)

print(" -> ".join(full_route))
