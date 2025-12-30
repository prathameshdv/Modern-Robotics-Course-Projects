# A* Path Planning with Obstacle Avoidance

This project implements a **graph-based path planning system** using the **A\* (A-star) search algorithm** to find the shortest collision-free path between two nodes in a 2D environment.

---

## Concepts Used

### Graph Representation
- The environment is modeled as a **weighted, undirected graph**.
- Nodes represent points in 2D space.
- Edges represent possible connections between nodes with an associated traversal cost.

---

### A* Search Algorithm
- Uses both **actual path cost (g)** and **heuristic estimate (h)** to guide the search.
- Prioritizes nodes with the lowest total cost `f = g + h`.
- Guarantees an optimal path when the heuristic is admissible.

---

### Heuristic Function
- Each node is assigned a precomputed heuristic value.
- The heuristic estimates the remaining cost to reach the goal node.
- Improves search efficiency compared to uninformed algorithms.

---

### Obstacle Modeling
- Obstacles are represented as **circular regions** in the 2D plane.
- Each obstacle is defined by a center point and a radius.

---

### Collision Detection
- Graph edges are checked for intersection with obstacles.
- Any edge intersecting an obstacle is removed before path planning.
- Ensures all valid paths are **collision-free**.

---

### Efficient Data Structures
- A **priority queue (min-heap)** is used to efficiently select the next node to expand.
- Cost tracking prevents unnecessary re-exploration of nodes.

---

### Path Reconstruction
- Once the goal is reached, the final path is reconstructed by backtracking.
- The computed path is saved as a sequence of node IDs.

---

## Output
- The final shortest path is written to a CSV file.
- The output can be used for visualization or further analysis.

<img width="768" height="579" alt="Screenshot from 2025-12-31 02-10-27" src="https://github.com/user-attachments/assets/9825d928-514c-4b55-bc2e-9fa9036c937a" />

---

## Summary
This project demonstrates how **A\*** search, **graph theory**, and **geometric collision checking** can be combined to solve a practical path planning problem in continuous space.

