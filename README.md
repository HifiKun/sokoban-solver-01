# Sokoban Solver — Push-Optimal Room Search

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/HifiKun/sokoban-solver-01/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/HifiKun/sokoban-solver-01/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver for make.fun's [Build the best Sokoban solver, 21 November 2026](https://make.fun/bounty/sokoban-solver-01-run-20261121) bounty. One program (`main.ts`, Node 22, no dependencies) answers all 5 instances of a set in one run.

## How it works

The generated levels are a hallway with one-square-wide doors into rooms of 3–4 boxes. A box pushed through a door can never come back, so every room is independent — the push-optimal total is the sum of the rooms' optima.

1. **Decompose**: find the hallway row (the player's row), split the remaining floor into room components, one door per room.
2. **Solve each room push-optimally** with A* over pushes (state = sorted box cells + the player's reachable region), a min-assignment heuristic over BFS box→goal distances, dead-square pruning via reverse pull reachability, and 2×2 freeze pruning. If the push-optimal finish strands the player inside the room, it re-solves requiring the exit to stay reachable.
3. **Stitch**: shortest BFS walks between pushes and between rooms (walks cost no pushes, so only the push count matters). Rooms are visited nearest-first.

Everything is budgeted by expansions, never by the clock, so two runs write identical bytes.

Practice set score: **1,000,000 / 1,000,000** — every instance solved in exactly P\* pushes (13, 16, 29, 47, 60).

## Run it

```sh
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

Reads `/input/<id>/level.txt`, writes `/output/<id>/solution.txt` (moves in `u/d/l/r`).

---

<sub>Entered on [make.fun](https://make.fun/?ref=6a1ay90r&utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
