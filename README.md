# Pathfinding in Julia

A Julia implementation of classical pathfinding algorithms on grid maps.

The project reads map files, models obstacles and terrain costs, then compares several search strategies from uninformed search to heuristic search.

## Implemented Algorithms

- Breadth-First Search
- Dijkstra
- A*
- Greedy Best-First Search
- Weighted A*

The implementation also includes utility functions for reading map files, computing movement costs, finding valid neighbors, reconstructing paths, and printing the resulting route.

## Map Model

Maps are loaded from text files in `dat/`. The reader skips the file header until it finds the `map` line, then builds a character matrix.

Supported terrain behavior:

- `@`: obstacle, not traversable
- `.`: normal traversable cell, cost `1`
- `S`: slower terrain, cost `5`
- `W`: water-like terrain, cost `8`

Movement is 4-directional: up, down, left, and right.

## Repository Structure

```text
.
├── dat/
│   ├── Paris_2_1024.map
│   ├── Sydney_2_512.map
│   └── maze512-1-0.map
└── src/
    └── Path.jl
```

## Requirements

- Julia
- `DataStructures.jl`

Install the required package from the Julia REPL:

```julia
using Pkg
Pkg.add("DataStructures")
```

## Usage

Clone the repository:

```bash
git clone https://github.com/Bocoum1/Pathfinding.git
cd Pathfinding
```

Open Julia and load the source file:

```julia
include("src/Path.jl")
```

Run an algorithm on one of the bundled maps:

```julia
start = (333, 14)
goal = (312, 15)
map_file = "dat/maze512-1-0.map"

algobfs(map_file, start, goal)
algoDijkstra(map_file, start, goal)
algoAstar(map_file, start, goal)
algoGlouton(map_file, start, goal)
```

Run Weighted A*:

```julia
algo_wAstar(map_file, start, goal; mode=2, w=1.5)
```

## Weighted A* Modes

The `algo_wAstar` function supports several weighting strategies through `mode`:

- `mode=1`: weighted blend of path cost and heuristic
- `mode=2`: `f(n) = g(n) + w * h(n)`
- `mode=3`: dynamic weight that decreases as more states are evaluated

## Example Output

Each algorithm prints:

- the selected algorithm
- the distance or path cost
- the number of evaluated states
- the reconstructed path as coordinates

## Notes

This project is focused on algorithmic clarity rather than UI or visualization. A natural next step would be to add benchmark tables and path visualizations for the bundled maps.

## Roadmap

- Add benchmark results comparing evaluated states and path costs.
- Add image-based path visualization.
- Add tests for map parsing and path reconstruction.
- Package the code as a small Julia module.
