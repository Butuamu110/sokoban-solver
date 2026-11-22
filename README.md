# Sokoban Solver — @rojanon11

[![badge](https://make.fun/api/badge/Butuamu110/sokoban-solver)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![badge](https://make.fun/api/badge/Butuamu110/sokoban-solver)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![badge](https://make.fun/api/badge/Butuamu110/sokoban-solver)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver in a single self-contained `main.ts` (plain
JavaScript, Node 22, no dependencies). It searches in **push space** with A\*:
a state is the sorted box positions plus the canonical (smallest) cell of the
player's reachable region, and the heuristic is the sum over boxes of a
precomputed static push-distance to the nearest goal (reverse pull-BFS,
minimum over goals). Dead squares (no goal reachable) and 2×2 block deadlocks
are pruned.

Two distinguishing mechanisms:

- **Tunnel macros** — when a push lands a box in a 1-wide corridor (both
  perpendicular neighbours are walls), the only useful continuation is
  pushing it straight through, so the whole run through the tunnel is
  generated as a single expansion (stopping on goals or at the tunnel exit).
- **Push ordering** — generated pushes are ordered deterministically: pushes
  landing a box on a goal first, then pushes reducing the heuristic most,
  then pushes whose stance leaves the player in the largest reachable
  region, so A\* tie-breaking reaches goal states earlier.

The solver also exploits the level structure: the player's hallway row is
removed to split the board into rooms, each room is solved push-optimally on
its own (the player may stand in the hallway, boxes never enter it), and the
per-room push sequences are concatenated with hallway walks — per-room
optima sum to the level optimum. A whole-level push A\* is kept as a
fallback. Search is budgeted purely by deterministic expansion caps: the
program never reads the clock and uses no randomness, so repeated runs are
byte-identical.

Run it with `MAKEFUN_INPUT_DIR` / `MAKEFUN_OUTPUT_DIR` set (defaults
`/input` and `/output`); it writes `<id>/solution.txt` for every instance,
including instances it cannot solve (empty file).
