# Drone Relief Delivery System

A SWI-Prolog project that plans supply routes across a network of Sri Lankan
towns. It compares graph-search algorithms, tracks a drone's location and
battery during a session, and manages deliveries and temporary no-fly zones.

## Run the menu

Install [SWI-Prolog](https://www.swi-prolog.org/) and run the project from this
directory:

```powershell
swipl -s final_project.pl
```

The menu starts the drone at Ratnapura with a full battery. Choose an option by
entering its number followed by a period, such as `1.`. The menu lets you:

1. View pending delivery locations with a route, distance, battery cost, and
   package weight.
2. Select a delivery, review its route and battery requirement, and confirm it.
3. Compare DFS, BFS, and A* for a chosen start and destination.
4. Block a known road for the current session.
5. Unblock a road.
6. List blocked roads.
7. Reset the drone to Ratnapura with a full battery.
8. Exit.

Locations are Prolog atoms; enter capitalized names in single quotes, for
example `'Colombo'.` Roads entered for blocking or unblocking use the same
format. The network starts with the Avissawella–Kandy route blocked.

## Search and energy model

The network treats roads as bidirectional. Each `edge/3` stores a raw energy
cost, while `road_km/3` stores an approximate distance used for display. A
full battery covers 500 raw energy units, so route energy is displayed as a
percentage of the 100% battery capacity. Delivery package weights are
displayed separately from route energy.

- `dfs/4` finds routes with depth-first search.
- `bfs/4` finds a route with the fewest legs and reports that route's energy
  cost.
- `astar/4` searches using the configured heuristic estimates.
- `plan_tour/5` visits targets in the supplied order, then returns to the start.
- `compare_algorithms/2` displays the route metrics for all three algorithms.

The default delivery points are Colombo (25 kg), Hambantota (40 kg), and
Badulla (30 kg). A successful delivery updates the drone's location and
battery and marks that camp as served for the session. Resetting the drone
refills its battery and returns it to Ratnapura while keeping the delivery
history; starting a new menu session clears that history.

## Query the project

Consult the file from the SWI-Prolog prompt to load its predicates without
starting the menu:

```prolog
?- consult('final_project.pl').
true.

?- astar('Ratnapura', 'Colombo', Path, Cost).
Path = ['Ratnapura', 'Avissawella', 'Colombo'],
Cost = 81.

?- compare_algorithms('Ratnapura', 'Colombo').
```

The map, heuristic estimates, road distances, delivery points, and initial
blocked route are defined near the top of [final_project.pl](final_project.pl).
