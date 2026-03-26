# Hospital Pathfinding in Heidelberg

This project compares Dijkstra's algorithm and A* search for emergency route planning in Heidelberg, Germany.

## Features
* **Map Data**: Uses OpenStreetMap via the OSMnx library.
* **Pathfinding**: It includes implementations of Dijkstra and A* to find the shortest driving routes.
* **Performance**: Memory and execution time tracking using tracemalloc and time.
* **Validation**: Computed distances are checked against NetworkX shortest path results.

## Scope
* City: Heidelberg, Germany
* Scenarios: 5 start locations
* Targets: 5 hospitals

## Setup
Install the requirements:
```bash
pip install -r requirements.txt
```
Run the script: 
```bash
python main.py
