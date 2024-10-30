import heapq
import math
import random
import matplotlib.pyplot as plt
import networkx as nx


# 创建随机网络
def create_random_network(num_nodes=10):
    nodes = {chr(65 + i): (random.uniform(0, 100), random.uniform(0, 100)) for i in range(num_nodes)}
    distances = {}
    for node1, (x1, y1) in nodes.items():
        distances[node1] = []
        for node2, (x2, y2) in nodes.items():
            if node1 != node2:
                distance = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
                distances[node1].append((distance, node2))
        distances[node1].sort()
    network = {node: [] for node in nodes}
    for node, dist_list in distances.items():
        for _, neighbor in dist_list[:3]:  # 连接最近的3个节点
            network[node].append(neighbor)
    return network, nodes


# Dijkstra 算法计算最短路径
def dijkstra(network, attr_map, origin, destination):
    distances = {node: math.inf for node in network}
    distances[origin] = 0
    previous_nodes = {node: None for node in network}
    priority_queue = [(0, origin)]

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)
        if current_node == destination:
            break
        for neighbor in network[current_node]:
            if (current_node, neighbor) in attr_map:
                distance = attr_map[(current_node, neighbor)]
                new_distance = current_distance + distance
                if new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    previous_nodes[neighbor] = current_node
                    heapq.heappush(priority_queue, (new_distance, neighbor))

    path = []
    node = destination
    while node is not None:
        path.insert(0, node)
        node = previous_nodes[node]

    return path if distances[destination] != math.inf else None


# 创建长度和速度映射
def create_attr_maps(network, nodes):
    length_map = {}
    speed_map = {}
    for node, neighbors in network.items():
        x1, y1 = nodes[node]
        for neighbor in neighbors:
            x2, y2 = nodes[neighbor]
            distance = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
            length_map[(node, neighbor)] = distance
            speed_map[(node, neighbor)] = random.uniform(30, 60)  # 随机速度
    return length_map, speed_map


# 成本函数
def calculate_cost(length, speed, flow):
    return length / speed + 0.1 * flow


# 更新成本函数的映射
def update_cost(cost_function_map, flow_map):
    updated_map = {}
    for key, cost_function in cost_function_map.items():
        flow = flow_map.get(key, 0)
        updated_map[key] = cost_function(flow)
    return updated_map


# 分配流量
def assign_flow(network, cost_function_map, od_pairs):
    flow_map = {(u, v): 1 for u, v in list_all_network_links(network)}  # 为每条边初始化少量流量
    for od_pair in od_pairs:
        new_flow_map = single_od_assign(network, cost_function_map, od_pair, flow_map)
        flow_map = combine_flow_maps(flow_map, new_flow_map)
    return flow_map


# 单对OD流量分配
def single_od_assign(network, cost_function_map, od_pair, base_flow_map):
    origin, destination, flow = od_pair
    cost_map = update_cost(cost_function_map, base_flow_map)
    shortest_path = dijkstra(network, cost_map, origin, destination)

    # 检查最短路径是否存在
    if shortest_path is None:
        print(f"No path found between {origin} and {destination}")
        return base_flow_map  # 如果路径不存在，返回基础流量图

    flow_map = load_flow(network, shortest_path, flow)

    for i in range(50):
        cost_map = update_cost(cost_function_map, combine_flow_maps(base_flow_map, flow_map))
        shortest_path = dijkstra(network, cost_map, origin, destination)
        if shortest_path is None:
            print(f"No path found during iteration {i} for {origin} to {destination}")
            break  # 跳过该 OD 对的后续计算
        flow_map = scale_flow_map(flow_map, 0.95)
        new_flow_map = load_flow(network, shortest_path, flow * 0.05)
        flow_map = combine_flow_maps(flow_map, new_flow_map)

    return flow_map


# 合并流量图
def combine_flow_maps(map1, map2):
    combined_map = {}
    for key in set(map1) | set(map2):
        combined_map[key] = map1.get(key, 0) + map2.get(key, 0)
    return combined_map


# 缩放流量图
def scale_flow_map(map1, scale):
    return {key: value * scale for key, value in map1.items()}


# 加载流量
def load_flow(network, path, flow):
    flow_map = {}
    links = set(zip(path[:-1], path[1:]))
    for link in list_all_network_links(network):
        flow_map[link] = flow if link in links else 0
    return flow_map


# 获取所有网络链接
def list_all_network_links(network):
    return [(node, neighbor) for node, neighbors in network.items() for neighbor in neighbors]


# 创建并绘制网络
def draw_network(network, flow_map, node_positions):
    G = nx.DiGraph()
    for node, neighbors in network.items():
        G.add_node(node, pos=node_positions[node])
        for neighbor in neighbors:
            G.add_edge(node, neighbor, flow=flow_map.get((node, neighbor), 0))

    pos = {node: node_positions[node] for node in G.nodes()}
    nx.draw(G, pos, with_labels=True, node_size=500, node_color="skyblue", font_size=10, font_weight="bold",
            arrows=True)

    edge_labels = {(u, v): f"{d['flow']:.1f}" for u, v, d in G.edges(data=True)}
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color="red", font_size=8)

    plt.title("Random Traffic Network with Flow Distribution")
    plt.show()


# 主程序
network, node_positions = create_random_network(num_nodes=10)
length_map, speed_map = create_attr_maps(network, node_positions)

# 创建成本函数映射
cost_function_map = {
    link: lambda flow, l=length_map[link], s=speed_map[link]: calculate_cost(l, s, flow)
    for link in length_map
}

# 随机生成更多 OD 对和流量
od_pairs = [(random.choice(list(network.keys())), random.choice(list(network.keys())), random.randint(50, 150)) for _ in
            range(15)]

# 分配流量
flow_map = assign_flow(network, cost_function_map, od_pairs)
print("Flow Map:", flow_map)

# 绘制网络
draw_network(network, flow_map, node_positions)
