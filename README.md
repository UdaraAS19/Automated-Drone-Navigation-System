# Drone Relief Delivery System

The project is implemented in [final_project.pl](final_project.pl). It models
relief routes, compares DFS, BFS, and A* searches, tracks drone battery and
deliveries, and provides an interactive menu for delivery and no-fly-zone
management.

## Run

Start the interactive menu with SWI-Prolog:

```powershell
swipl -s final_project.pl
```

To load the knowledge base without starting the menu, consult the file and call
predicates such as `astar/4`, `compare_algorithms/2`, or `plan_tour/5`.

Locations and delivery points are defined near the top of the file. The menu
starts the drone at Ratnapura with a full battery.
