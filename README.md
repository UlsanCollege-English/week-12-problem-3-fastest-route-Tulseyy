[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/Dd1WvBHx)
# hw03 – Fastest Route with Dijkstra

## Story

You are building a helper for a **festival delivery cart** inside a large convention center.  
Corridors connect booths. Each corridor has a **travel time in minutes** (some are longer, some are shorter).

You want to find the **fastest route** from one booth to another.  
This is a **weighted graph** problem. We will use **Dijkstra’s algorithm**.

---

## Task

Write a function:

```python
dijkstra_shortest_path(graph, start, goal)
```

- `graph`: dictionary `node -> list of (neighbor, weight)`

    - `weight` is a positive integer travel time (minutes).

- `start`: starting node (string)

- `goal`: target node (string)

Return:

- `(path, total_cost)` where:

    - `path` is a list `[start, ..., goal]` with minimum total cost

    - `total_cost` is the sum of weights along that path

- If `start` or `goal` is not in `graph`, or `goal` is unreachable:

    - return `([], None)`

Constraints

- At most 200 nodes and 1000 edges.

- All weights are positive integers.

- Expected time complexity:

    - Use a priority queue (heap) → about O((V + E) log V).

---

## 8 Steps of Coding – Scaffold (hw03)
For hw03, scaffolding is lighter:

1. Step 1–3: Problem framing (do briefly)

    - Step 1: Write 1–2 sentences: what does the function compute?

    - Step 2: Re-phrase: “Find the path with the smallest sum of weights.”

    - Step 3: List inputs (graph, start, goal), outputs (path, total_cost), and structures (dist, parent, heap).
2. Step 4–8: You drive

    - Step 4: Plan how you relax edges and update distances.

    - Step 5: Write pseudocode for Dijkstra.

    - Step 6: Implement with heapq.

    - Step 7: Debug with small graphs (draw them).
    - Step 8: Explain to yourself why the complexity is about O((V + E) log V).

---
## Hints
1. Use dist dictionary with default float("inf") for unknown distances.

1. Use (current_dist, node) tuples in a heapq priority queue.

1. Skip work if current_dist > dist[node]. This avoids stale entries.

---

## How to Run Tests
```
python -m pytest -q
```    
---

## FAQ
Q1: Do I need to rebuild the path?
A1: Yes. Use a `parent` dictionary and walk from `goal` back to `start`, then reverse.

Q2: What if `start == goal`?
A2: Return `([start], 0)`.

Q3: What if `start` or `goal` is missing from `graph`?
A3: Return `([], None)`.

Q4: What about negative weights?
A4: Not allowed here. All weights are positive. Dijkstra does not work with negative weights.

Q5: Big-O expectation?
A5: O((V + E) log V) if you use `heapq` correctly. A simple list for the priority queue would be slower.

Q6: How can I debug wrong paths?
A6: Print `dist` and `parent` after the algorithm, and compare with hand-calculated values on a small graph.

Q7: What are common pitfalls?
A7: Not skipping stale `(dist, node)` pairs, not handling unreachable goals, and mixing up weight vs dist.