import heapq
import math
from functools import partial
from typing import NamedTuple
from shapely import LineString
from shapely import Point as Pt
from sortedcontainers import SortedList
import networkx as nx
import matplotlib.pyplot as plt


class Point(NamedTuple):
    x: float
    y: float

    def __add__(self, other: 'Point') -> 'Point':
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other: 'Point') -> 'Point':
        return Point(self.x - other.x, self.y - other.y)


def add_edge(g, vertex1, vertex2):
    if vertex1 not in g:
        g[vertex1] = []
    g[vertex1].append(vertex2)

    if vertex2 not in g:
        g[vertex2] = []
    g[vertex2].append(vertex1)


def find_edges(g, v):
    if v in g:
        return g[v]
    else:
        return []


def angle_and_distance(point1, point2):
    distance = math.dist(point1, point2)
    c_length = abs(point2.y - point1.y)
    if distance == 0:
        distance = 0.001
    angle = math.asin(c_length / distance)
    if (point2.x - point1.x > 0) and (point2.y - point1.y == 0):
        return angle, distance
    if (point2.x - point1.x < 0) and (point2.y - point1.y < 0):
        angle = ((math.pi / 2) - angle) + (math.pi / 2)
    elif (point2.x - point1.x >= 0) and (point2.y - point1.y >= 0):
        angle = ((math.pi / 2) - angle) + (math.pi * 3 / 2)
    elif (point2.x - point1.x < 0) and (point2.y - point1.y >= 0):
        angle = math.pi + angle
    return angle, distance


def segments_intersect(p1, q1, p2, q2):
    line1 = LineString([p1, q1])
    line2 = LineString([p2, q2])
    return line1.crosses(line2)


def point_on_line(point, start_point, end_point):
    line = LineString([start_point, end_point])
    point_shapely = Pt(point.x, point.y)
    return line.distance(point_shapely) == 0


def distance_from_point_to_edge(p, edge):
    line = LineString([edge[0], edge[1]])
    point = Pt(p.x, p.y)
    return point.distance(line)


def find_subarray_with_point(obstacles, target_point):
    for subarray in obstacles:
        if target_point in subarray:
            return subarray
    return None


def segments_angle(base_point, point1, point2):
    vector1 = (point1[0] - base_point[0], point1[1] - base_point[1])
    vector2 = (point2[0] - base_point[0], point2[1] - base_point[1])
    dot_product = vector1[0] * vector2[0] + vector1[1] * vector2[1]
    magnitude1 = math.sqrt(vector1[0] ** 2 + vector1[1] ** 2)
    magnitude2 = math.sqrt(vector2[0] ** 2 + vector2[1] ** 2)
    cos_angle = dot_product / (magnitude1 * magnitude2)
    angle_rad = math.acos(cos_angle)
    return angle_rad


def check_edge_inside(base_point, point1, point2, point_vis):
    angle_part_1 = segments_angle(base_point, point1, point_vis)
    angle_part_2 = segments_angle(base_point, point2, point_vis)
    angle = segments_angle(base_point, point1, point2)
    angle_sum = angle_part_1 + angle_part_2
    if angle_sum > math.pi:
        angle_sum = 2 * math.pi - angle_sum
    return abs(angle - angle_sum) < 0.001


def check_non_inner_edge(start_point, w_j):
    subarray = find_subarray_with_point(obstacles, start_point)
    if subarray and len(subarray) > 3 and w_j in subarray and w_j not in find_edges(graph, start_point):
        index = subarray.index(start_point)
        left = index - 1
        right = index + 1
        if left < 0:
            left += len(subarray)
        if right == len(subarray):
            right -= len(subarray)
        first_edge_point = subarray[left]
        second_edge_point = subarray[right]
        if check_edge_inside(start_point, first_edge_point, second_edge_point, w_j):
            return False
    return True


def is_clockwise(segment_start, segment_end, point_out):
    v1_x = segment_end.x - segment_start.x
    v1_y = segment_end.y - segment_start.y

    v2_x = point_out.x - segment_start.x
    v2_y = point_out.y - segment_start.y

    cross_product = v1_x * v2_y - v1_y * v2_x

    if cross_product < 0:
        return 1  # point is in the right half
    elif cross_product > 0:
        return -1  # point is in the left half
    else:
        return 0  # point is on the segment


def check_direct_connection(start, finish):
    for obs in obstacles:
        for k in range(len(obs)):
            if segments_intersect(obs[k], obs[(k + 1) % len(obs)], start, finish):
                return False
    return True


def check_intersect_with_obstacle(start_point, w_i):
    polygon = find_subarray_with_point(obstacles, w_i)
    if not polygon:
        return False
    n = len(polygon)
    for j in range(n):
        next_j = (j + 1) % n
        if segments_intersect(start_point, w_i, polygon[j], polygon[next_j]):
            return True
    return False


def visible(tree, w, j, visible_points_array, start_point):
    if len(tree) == 0:
        if check_intersect_with_obstacle(start_point, w[j]) or check_intersect_with_obstacle(w[j], start_point):
            return False
        if not check_non_inner_edge(start_point, w[j]):
            return False
        visible_points_array.append(w[j])
        return True
    if j == 0 or not point_on_line(w[j - 1], start_point, w[j]):
        p1 = edges_array[tree[0][1]][0]
        p2 = edges_array[tree[0][1]][1]
        if segments_intersect(start_point, w[j], p1, p2):
            return False
        else:
            if not check_non_inner_edge(start_point, w[j]):
                return False
            if check_intersect_with_obstacle(start_point, w[j]) or check_intersect_with_obstacle(w[j], start_point):
                return False
            visible_points_array.append(w[j])
            return True
    else:
        if visible_points_array and (w[j - 1] == visible_points_array[-1]):
            for h in tree:
                p1 = edges_array[h[1]][0]
                p2 = edges_array[h[1]][1]
                if segments_intersect(p1, p2, w[j - 1], w[j]):
                    return False
            if not check_non_inner_edge(w[j - 1], w[j]):
                return False
            if check_intersect_with_obstacle(start_point, w[j]) or check_intersect_with_obstacle(w[j], start_point):
                return False
            visible_points_array.append(w[j])
            return True
        else:
            return False


def get_index_in_encountered_edges(element):
    if element in edges_dict:
        return edges_dict[element]
    else:
        return edges_dict[element[1], element[0]]


def find_visible_points(start_point, w, visible_graph):
    tree = SortedList()
    far_right_point = Point(max(p.x for p in all_points) + 1, start_point.y)

    temp_set = set()
    for vertex, neighbors in graph.items():
        for neighbor in neighbors:
            if vertex != start_point and neighbor != start_point:
                edge = tuple(sorted([vertex, neighbor]))
                if segments_intersect(start_point, far_right_point, vertex, neighbor):
                    temp_set.add(edge)

    for e in temp_set:
        dist = distance_from_point_to_edge(start_point, e)
        ind = get_index_in_encountered_edges(e)
        tree.add((dist, ind))

    visib_points = []
    for i in range(len(w)):
        if w[i] == start_point:
            continue
        visible(tree, w, i, visib_points, start_point)
        if i != len(w) - 1:
            adjacent_points = find_edges(graph, w[i])
            for p in adjacent_points:
                orient = is_clockwise(start_point, w[i], p)
                if orient == 1:
                    if start_point != p:
                        dist = distance_from_point_to_edge(start_point, (w[i], p))
                        ind = get_index_in_encountered_edges((w[i], p))
                        tree.add((dist, ind))
                elif orient == -1:
                    if (w[i], p) in edges_dict or (p, w[i]) in edges_dict:
                        dist = distance_from_point_to_edge(start_point, (w[i], p))
                        ind = get_index_in_encountered_edges((w[i], p))
                        tree.discard((dist, ind))
    for edge in visib_points:
        add_edge(visible_graph, start_point, edge)


def visibility_graph(visible_graph):
    for point in all_points:
        sorted_points = sorted(obstacle_points, key=partial(angle_and_distance, point))
        find_visible_points(point, sorted_points, visible_graph)


def distance_calc(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def dijkstra(vis_graph, start, end):
    distances = {v: float('infinity') for v in vis_graph}
    distances[start] = 0

    previous_vertices = {v: None for v in vis_graph}
    priority_queue = [(0, start)]

    while priority_queue:
        current_distance, current_vertex = heapq.heappop(priority_queue)

        if current_vertex == end:
            break

        for n in vis_graph[current_vertex]:
            distance = distance_calc(current_vertex, n)
            new_distance = current_distance + distance

            if new_distance < distances[n]:
                distances[n] = new_distance
                previous_vertices[n] = current_vertex
                heapq.heappush(priority_queue, (new_distance, n))

    path = []
    current_vertex = end
    while previous_vertices[current_vertex] is not None:
        path.append((previous_vertices[current_vertex], current_vertex))
        current_vertex = previous_vertices[current_vertex]
    path.reverse()
    return path


def plot_graphs():
    G = nx.Graph()
    VG = nx.Graph()

    for vertex, neighbors in graph.items():
        for neighbor in neighbors:
            G.add_edge(vertex, neighbor)

    for vertex, neighbors in visible_graph.items():
        for neighbor in neighbors:
            VG.add_edge(vertex, neighbor)

    pos_G = {node: node for node in G.nodes()}
    pos_VG = {node: node for node in VG.nodes()}
    node_colors = ["lightblue" if node in VG.nodes() else "lightgray" for node in VG.nodes()]
    plt.figure(figsize=(8, 6))

    nx.draw_networkx_nodes(VG, pos_VG, node_size=500, node_color=node_colors)
    nx.draw_networkx_edges(VG, pos_VG, edgelist=VG.edges(), edge_color='gray')
    nx.draw_networkx_edges(G, pos_G, edgelist=G.edges(), edge_color='red')
    nx.draw_networkx_edges(VG, pos_VG, edgelist=shortest_path_edges, edge_color='green', width=2)
    nx.draw_networkx_labels(G, pos_G, labels={node: str(node) for node in G.nodes()}, font_size=10, font_weight="bold")
    specific_labels = {S: f"S", T: f"T"}
    nx.draw_networkx_labels(VG, pos_VG, labels=specific_labels, font_size=12, font_color="green", font_weight="bold")
    plt.title("Graph Visualization")
    plt.show()


if __name__ == "__main__":
    S = Point(1, 2)
    T = Point(9, 2)
    obstacles = [
        [Point(3, 1), Point(4, 2), Point(5, 2), Point(2, 4)],
        [Point(6, 0), Point(8, 1), Point(7, 2)]
    ]

    graph = {}
    visible_graph = {}
    encountered_edges_set = set()

    all_points = [S, T]
    obstacle_points = []
    for obstacle in obstacles:
        all_points.extend(obstacle)
        obstacle_points.extend(obstacle)

    for obstacle in obstacles:
        for i in range(len(obstacle)):
            add_edge(graph, obstacle[i], obstacle[(i + 1) % len(obstacle)])
            encountered_edges_set.add((obstacle[i], obstacle[(i + 1) % len(obstacle)]))

    edges_dict = {}
    edges_array = []

    for index, tup in enumerate(encountered_edges_set):
        edges_dict[tup] = index
        edges_array.append(tup)

    if check_direct_connection(S, T):
        add_edge(visible_graph, S, T)
    else:
        visibility_graph(visible_graph)

    shortest_path_edges = dijkstra(visible_graph, S, T)

    plot_graphs()
