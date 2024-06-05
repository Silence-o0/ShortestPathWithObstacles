import math
from functools import partial
from typing import NamedTuple

from shapely import LineString
from shapely import Point as Pt
from sortedcontainers import SortedList
import networkx as nx
import matplotlib.pyplot as plt


# Це може бути будь-який клас точки. Тут це просто tuple
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


def find_edges(graph, vertex):
    if vertex in graph:
        return graph[vertex]
    else:
        return []


def angle_and_distance(point1, point2):
    vector = point2 - point1
    distance = math.dist(point1, point2)
    angle = math.asin(-vector[1] / distance)
    if angle < 0:
        angle += 2 * math.pi
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


def visible(sorted_edges, tree, w, i, visible_points_array, start_point):
    if len(tree) == 0:
        visible_points_array.append(w[i])
        return True
    if i == 0 or not point_on_line(w[i - 1], S, w[i]):
        p1 = sorted_edges[tree[0]][0]
        p2 = sorted_edges[tree[0]][1]
        if segments_intersect(start_point, w[i], p1, p2):
            return False
        else:
            visible_points_array.append(w[i])
            return True
    else:
        if visible_points_array and (w[i - 1] == visible_points_array[-1]):
            for h in tree:
                p1 = sorted_edges[h][0]
                p2 = sorted_edges[h][1]
                if segments_intersect(p1, p2, w[i - 1], w[i]) == True:
                    return False
            visible_points_array.append(w[i])
            return True
        else:
            return False


def is_clockwise(segment_start, segment_end, edge_end):
    # Вектор від пheapq.heappopочатку відрізка до його кінця (v1)
    v1_x = segment_end.x - segment_start.x
    v1_y = segment_end.y - segment_start.y

    # Вектор від кінця відрізка до іншого кінця ребра (v2)
    v2_x = edge_end.x - segment_end.x
    v2_y = edge_end.y - segment_end.y

    # Векторний добуток v1 та v2
    cross_product = v1_x * v2_y - v1_y * v2_x

    # Повертаємо 1, якщо v2 лежить по часовій стрілці відносно v1,
    # -1 якщо він лежить проти часової стрілки, і 0 якщо вони співпадають
    if cross_product > 0:
        return 1
    elif cross_product < 0:
        return -1
    else:
        return 0


def visible_points(start_point, graph, w, visible_graph, all_points):
    encountered_edges = set()

    # Знайдемо усі ребра, що перетинають промінь
    far_right_point = Point(max(point.x for point in all_points),
                            start_point.y)

    for vertex, neighbors in graph.items():
        for neighbor in neighbors:
            # Створюємо кортеж, що представляє ребро
            edge = tuple(sorted([vertex, neighbor]))
            if segments_intersect(
                    start_point, far_right_point,
                    vertex, neighbor
            ):
                encountered_edges.add(edge)

    # print(encountered_edges)

    sorted_edges = sorted(encountered_edges,
                          key=lambda edge: distance_from_point_to_edge(S,
                                                                       edge))
    # print(sorted_edges)
    # тут ці encountered_edges мають бути відсортовані, але це реалізуй сама
    # Створюємо кучу, в якій зберігаються індекси в масиві encountered_edges.
    # Зараз там усі індекси від 0 до len(encountered_edges)-1.
    tree = SortedList(range(len(sorted_edges)))

    # Проходимось по точках
    # for point in w:
    #     if point == start_point:
    #         нема сенсу обробляти цю точку
    # continue

    visib_points = []
    for i in range(len(w)):
        if w[i] == start_point:
            continue
        visible(sorted_edges, tree, w, i, visib_points, start_point)
        if i != len(w) - 1:
            adjacent_points = find_edges(graph, w[i])
            for p in adjacent_points:
                orient = is_clockwise(S, w[i], p)
                if orient == 1:
                    sorted_edges.append((w[i], p))
                    tree.add(len(sorted_edges) - 1)
                elif orient == -1:
                    if Point(w[i], p) in sorted_edges:
                        index = sorted_edges.index(Point(w[i], p))
                        tree.discard(index)
                    elif Point(p, w[i]) in sorted_edges:
                        index = sorted_edges.index(Point(p, w[i]))
                        tree.discard(index)

    print(visib_points)
    for edge in visible_graph:
        add_edge(visible_graph, S, edge)

        # тут ти маєш видалити ребра за їх індексами
        # проблема в тому, що треба отримувати індекс ефективно, тож
        # думай, як це зробити
        # щоб видалити ребро з індексом i: heapq.heappop(tree, i)

        # тут ти маєш перевірити чи точку видно. Те, що найлівіше в дереві,
        # в кучі tree[0]

        # тут new_edges - список ребер, які треба додати. Його зроби сама
        # new_edges = []
        # for edge in new_edges:
        #     index = len(new_edges)
        #     new_edges.append(edge)
        #     heapq.heappush(tree, index)


def visibility_graph(S, graph, sorted_points, visible_graph, all_points):
    for point in all_points:
        visible_points(point, graph, sorted_points, visible_graph, all_points)


if __name__ == "__main__":
    S = Point(1, 2)
    T = Point(9, 2)
    obstacles = [
        [Point(3, 1), Point(4, 2), Point(5, 2), Point(2, 4)],
        [Point(6, 0), Point(8, 1), Point(7, 2)]
    ]

    graph = {}

    visible_graph = {}

    all_points = [S, T]
    obstacle_points = []
    for obstacle in obstacles:
        all_points.extend(obstacle)
        obstacle_points.extend(obstacle)

    sorted_points = sorted(obstacle_points, key=partial(angle_and_distance, S))

    # print("Sorted points by angle and distance:", sorted_points)

    for obstacle in obstacles:
        for i in range(len(obstacle)):
            add_edge(graph, obstacle[i], obstacle[(i + 1) % len(obstacle)])

    visibility_graph(S, graph, sorted_points, visible_graph, all_points)

    G = nx.Graph()

    for vertex, neighbors in graph.items():
        for neighbor in neighbors:
            G.add_edge(vertex, neighbor)

    for vertex, neighbors in visible_graph.items():
        for neighbor in neighbors:
            G.add_edge(vertex, neighbor)

    pos = {node: node for node in G.nodes()}

    plt.figure(figsize=(8, 6))
    nx.draw(G, pos, with_labels=True, node_size=500, node_color="lightblue",
            font_size=10, font_weight="bold",
            edge_color="gray")

    plt.title("Graph Visualization")
    plt.show()
    #
    # for vertex, edges in graph.items():
    #     print(f"{vertex}: {edges}")
    #
    # fig, ax = plt.subplots()
    # ax.plot(S[0], S[1], 'go', label='Point S')
    # ax.plot(T[0], T[1], 'ro', label='Point T')
    # for obstacle in obstacles:
    #     obstacle.append(obstacle[
    #                         0])  # Додати першу вершину в кінець, щоб замкнути перешкоду
    #     xs, ys = zip(*obstacle)  # Розпаковка координат
    #     ax.plot(xs, ys, 'b-')
    #
    # for vertex, neighbors in graph.items():
    #     for neighbor in neighbors:
    #         G.add_edge(vertex, neighbor)
    #
    # pos = {node: node for node in G.nodes()}
    #
    # plt.figure(figsize=(8, 6))
    # nx.draw(G, pos, with_labels=True, node_size=500, node_color="lightblue",
    #         font_size=10, font_weight="bold",
    #         edge_color="gray")
    # ax.set_aspect('equal')
    # ax.set_xlim(0, 10)
    # ax.set_ylim(0, 5)
    # ax.legend()
    # plt.show()