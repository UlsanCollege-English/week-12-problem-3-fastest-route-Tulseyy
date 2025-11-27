import pytest
from main import dijkstra_shortest_path


# Normal tests (4)


def test_simple_weighted_line():
    graph = {
        "A": [("B", 3)],
        "B": [("A", 3), ("C", 4)],
        "C": [("B", 4)],
    }
    path, cost = dijkstra_shortest_path(graph, "A", "C")
    assert path == ["A", "B", "C"]
    assert cost == 7


def test_choose_path_with_more_edges_but_lower_cost():
    graph = {
        "A": [("B", 10), ("C", 2)],
        "B": [("A", 10), ("D", 2)],
        "C": [("A", 2), ("D", 2)],
        "D": [("B", 2), ("C", 2)],
    }
    path, cost = dijkstra_shortest_path(graph, "A", "D")
    # Best path is A -> C -> D with cost 4
    assert path[0] == "A" and path[-1] == "D"
    assert cost == 4


def test_small_square_graph():
    graph = {
        "A": [("B", 1), ("C", 5)],
        "B": [("A", 1), ("D", 2)],
        "C": [("A", 5), ("D", 1)],
        "D": [("B", 2), ("C", 1)],
    }
    path, cost = dijkstra_shortest_path(graph, "A", "D")
    assert cost == 3  # A -> B -> D


def test_start_equals_goal():
    graph = {
        "A": [("B", 2)],
        "B": [("A", 2)],
    }
    path, cost = dijkstra_shortest_path(graph, "A", "A")
    assert path == ["A"]
    assert cost == 0


# Edge-case tests (3)


def test_missing_start_or_goal_returns_empty_and_none():
    graph = {"A": [("B", 1)], "B": [("A", 1)]}
    assert dijkstra_shortest_path(graph, "X", "B") == ([], None)
    assert dijkstra_shortest_path(graph, "A", "Y") == ([], None)


def test_unreachable_goal():
    graph = {
        "A": [("B", 1)],
        "B": [("A", 1)],
        "C": [("D", 2)],
        "D": [("C", 2)],
    }
    path, cost = dijkstra_shortest_path(graph, "A", "D")
    assert path == []
    assert cost is None


def test_graph_with_single_node():
    graph = {"Solo": []}
    path, cost = dijkstra_shortest_path(graph, "Solo", "Solo")
    assert path == ["Solo"]
    assert cost == 0


# Complex tests (3)


def test_larger_graph_multiple_routes():
    graph = {
        "Start": [("A", 2), ("B", 5)],
        "A": [("Start", 2), ("C", 4)],
        "B": [("Start", 5), ("C", 1), ("D", 7)],
        "C": [("A", 4), ("B", 1), ("End", 3)],
        "D": [("B", 7), ("End", 1)],
        "End": [("C", 3), ("D", 1)],
    }
    path, cost = dijkstra_shortest_path(graph, "Start", "End")
    assert path[0] == "Start"
    assert path[-1] == "End"
    assert cost == 9  # Start -> A -> C -> End (2 + 4 + 3)


@pytest.mark.parametrize(
    "start,goal,expected_cost",
    [
        ("Start", "C", 6),
        ("Start", "D", 10),
        ("A", "End", 7),
    ],
)
def test_parametrized_shortest_costs(start, goal, expected_cost):
    graph = {
        "Start": [("A", 2), ("B", 5)],
        "A": [("Start", 2), ("C", 4)],
        "B": [("Start", 5), ("C", 1), ("D", 7)],
        "C": [("A", 4), ("B", 1), ("End", 3)],
        "D": [("B", 7), ("End", 1)],
        "End": [("C", 3), ("D", 1)],
    }
    path, cost = dijkstra_shortest_path(graph, start, goal)
    assert path[0] == start
    assert path[-1] == goal
    assert cost == expected_cost


def test_longer_chain_graph():
    # A chain with varying weights
    graph = {
        "P1": [("P2", 1)],
        "P2": [("P1", 1), ("P3", 2)],
        "P3": [("P2", 2), ("P4", 3)],
        "P4": [("P3", 3), ("P5", 4)],
        "P5": [("P4", 4)],
    }
    path, cost = dijkstra_shortest_path(graph, "P1", "P5")
    assert path == ["P1", "P2", "P3", "P4", "P5"]
    assert cost == 1 + 2 + 3 + 4
