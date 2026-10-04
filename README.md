# Automated Drone Relief Delivery System

**COU4303 – Artificial Intelligence | BSc (IT) Mini Project | Group 45**

A Prolog route-finding system that plans safe, battery-aware drone flights to deliver relief supplies between towns in Sri Lanka. The system finds routes with three search algorithms (**DFS**, **BFS** and **A\***), avoids blocked routes, checks battery and payload limits, and compares the algorithms by path and cost.

---

## 1. Introduction

### The problem

After a disaster, roads are often closed and relief camps in remote towns are hard to reach. A delivery drone has a limited battery and a limited payload, so it cannot fly just anywhere. Someone has to decide:

- Which route is the cheapest (in battery) to a camp?
- Is there enough battery for the flight **and the return trip to base**?
- What happens when a route becomes unsafe (bad weather, no-fly zone)?
- Can several camps be served in a single trip, and in what order?

### What this system does

The drone starts at a **base in Colombo** and delivers packages to six relief camps. Every flight is a round trip: the drone flies out, delivers, and returns to the base.

| Search algorithm | What it minimises | Role in the system |
|---|---|---|
| **DFS** (Depth-First Search) | Nothing – returns the first path it finds, and can list *all* paths | Shows every possible route and why the first one found is not always good |
| **BFS** (Breadth-First Search) | Number of roads used | Finds the route with the fewest roads |
| **A\*** | Total energy cost, guided by a heuristic | Finds the cheapest route; used for all deliveries and tours |

### Key features

- Route finding with **DFS, BFS and A\*** on a map with real air distances (km) and energy costs
- **Blocked routes**: block or unblock any road; every algorithm avoids blocked roads
- **Battery management**: a delivery or tour is denied if the battery is not enough for the full round trip
- **Payload limit**: a tour is denied if the total package weight is more than 100 kg
- **Multi-target tour**: give several camps at once; the system compares DFS/BFS/A\*, tries every visiting order, picks the cheapest, and returns to base
- **Algorithm comparison**: compare DFS, BFS and A\* between any two locations (path, energy, distance, battery, search effort)
- **Safe input**: names can be typed in any letter case, with or without quotes; wrong input never crashes the program

### The map

<img width="2816" height="1536" alt="SriLanka_Drone_Delivery_Network" src="https://github.com/user-attachments/assets/18a19a06-6774-424f-9536-3aa128069230" />


**Base:** Colombo  |  **Locations:** Colombo, Kandy, Badulla, Ratnapura, Galle, Hambantota, Polonnaruwa

| Delivery point | Package weight |
|---|---|
| Hambantota | 15 kg |
| Badulla | 15 kg |
| Kandy | 20 kg |
| Polonnaruwa | 15 kg |
| Galle | 15 kg |
| Ratnapura | 20 kg |

Rules used by the system:

- A full battery = **500 energy units = 100 %**. Battery percentages are rounded **up**, so the battery use is never under-estimated.
- Maximum payload per tour = **100 kg** (all six camps together are exactly 100 kg).
- The road **Ratnapura – Kandy** is blocked at start-up (bad weather). It can be unblocked with menu option 4.
- Each camp can be served only once until the drone is reset.

---

## 2. What you need

- **SWI-Prolog** (version 9.x recommended; this project was tested on 9.0.4). Download: https://www.swi-prolog.org/Download.html
- The source file: `COU4303_Group45.pl`

---

## 3. Step-by-step: how to run the project

### Step 1 – Install SWI-Prolog

- **Windows:** download the installer from the link above and run it. Tick **"Add swipl to the system PATH"** if the installer asks.
- **macOS:** install the `.dmg` from the website, or run `brew install swi-prolog`.
- **Linux (Ubuntu/Debian):** `sudo apt-get install swi-prolog`

Check the installation by opening a terminal (Command Prompt / PowerShell on Windows) and typing:

```
swipl --version
```

You should see a version number.

### Step 2 – Put the file in a folder

Save `COU4303_Group45.pl` in a folder that is easy to reach, for example `C:\Projects\` or `~/Projects/`.

### Step 3 – Load the program (choose ONE method)

**Method A – Terminal (recommended)**

1. Open a terminal in the folder that contains the file.
2. Type:
   ```
   swipl COU4303_Group45.pl
   ```
3. When you see the `?-` prompt, the file has loaded with no errors.

**Method B – SWI-Prolog window (Windows)**

1. Double-click `COU4303_Group45.pl` (it opens in SWI-Prolog), **or** open SWI-Prolog and choose **File → Consult…** and select the file.
2. Wait for the `?-` prompt.

**Method C – From inside SWI-Prolog**

```
?- consult('C:/Projects/COU4303_Group45.pl').
```

(Use forward slashes `/` in the path, even on Windows.)

### Step 4 – Start the system

At the `?-` prompt type the following and press Enter:

```
?- go.
```

The main menu appears:

```
==============================================
        DRONE RELIEF DELIVERY - MAIN MENU
==============================================
Current Location: Colombo   |   Battery Remaining: 100%
----------------------------------------------
1. View delivery locations (distance/route/battery cost/weight)
2. Deliver to a selected location
3. Block a route
4. Unblock a route
5. Show blocked routes
6. Plan a Multi-Target Tour
7. Reset drone (system reset)
8. Compare DFS / BFS / A* between two locations
9. Exit
Select an option (end with a period, e.g. 1.):
```

### Step 5 – Use the menu

> **Very important:** every answer you type must **end with a period `.`** and then press Enter.
> Example: `1.` or `Kandy.` – if you forget the period, the program keeps waiting for it.
> Location names are **not** case sensitive: `kandy.`, `Kandy.` and `'Kandy'.` all work.

| Option | What to type | What happens |
|---|---|---|
| **1** | `1.` | Shows every pending camp with its route, round-trip distance, battery cost and package weight |
| **2** | `2.` then a camp name, e.g. `Kandy.`, then `yes.` or `no.` | Shows the delivery details and, if the battery is enough, delivers after you confirm. Battery is reduced and the camp is marked as served |
| **3** | `3.` then two locations, e.g. `Ratnapura.` `Hambantota.` | Blocks the road between them. All algorithms now avoid it |
| **4** | `4.` then the same two locations | Unblocks the road |
| **5** | `5.` | Lists all blocked roads |
| **6** | `6.` then camps separated by commas, e.g. `Kandy, Badulla.` | Compares DFS/BFS/A\* for the tour, finds the cheapest visiting order, then asks `yes.`/`no.` before flying |
| **7** | `7.` | Resets the drone: battery 100 %, at Colombo, all deliveries cleared |
| **8** | `8.` then a start and a goal, e.g. `Colombo.` `Hambantota.` | Lists all DFS paths, then compares DFS, BFS and A\* |
| **9** | `9.` | Exits the menu |

### Step 6 – Try this quick walkthrough (about 3 minutes)

1. `1.` – see all six camps and their costs.
2. `8.` → `Colombo.` → `Hambantota.` – see that A\* finds the cheapest route (`[Colombo, Ratnapura, Hambantota]`, energy 75) while BFS picks a more expensive one (energy 116) with the same number of roads.
3. `3.` → `Ratnapura.` → `Hambantota.` – block a road. Repeat step 2 and notice that the route changes to `[Colombo, Galle, Hambantota]` (energy 103).
4. `4.` → `Ratnapura.` → `Hambantota.` – unblock it again.
5. `6.` → `Kandy, Badulla.` → `yes.` – fly a two-camp tour. The battery drops from 100 % to 72 %.
6. `2.` → `Galle.` → `yes.` – deliver to one more camp (battery 72 % → 50 %).
7. `6.` → `Hambantota, Polonnaruwa, Ratnapura.` – the battery is now 50 % but the tour needs 53 %, so you will see **"TOUR DENIED"**. Then use `7.` to reset the drone.
8. `9.` – exit.

### Step 7 – Optional: verify the A\* heuristic

Exit the menu (option 9) so you are back at the `?-` prompt, then type:

```
?- check_heuristics.
```

Expected result:

```
All 25 heuristic values are admissible (never overestimate).
```

This proves that A\* always returns the cheapest route.

### Step 8 – Leave SWI-Prolog

```
?- halt.
```

(or press `Ctrl + D`).

---

## 4. Understanding the output

**Option 8 (comparison) output**

```
dfs    roads used: 3 | energy: 120 | distance: 249 km | battery: 24% | search effort: 50 operations
bfs    roads used: 2 | energy: 116 | distance: 230 km | battery: 24% | search effort: 565 operations
astar  roads used: 2 | energy: 75 | distance: 158 km | battery: 15% | search effort: 208 operations

Lowest energy cost (75) : [astar]
Fewest roads used (2)  : [bfs,astar]
```

| Term | Meaning |
|---|---|
| roads used | Number of roads (flights between two towns) in the route |
| energy | Energy cost of the one-way route (500 units = full battery) |
| distance | Real air distance in km |
| battery | Energy cost as a percentage of a full battery |
| search effort | How much work the algorithm did to find the route |

**Why the results differ**

- **DFS** takes the first path it finds, so it is quick but not the cheapest.
- **BFS** finds the fewest roads, but the cheapest route is not always the one with the fewest roads.
- **A\*** uses the energy cost plus a heuristic, so it always finds the cheapest route.

---

## 5. Troubleshooting

| Problem | Solution |
|---|---|
| `'swipl' is not recognized` / `command not found` | SWI-Prolog is not in the PATH. Re-install and tick "Add to PATH", or start SWI-Prolog from the Start menu and use **File → Consult** |
| `source_sink ... does not exist` | The file name or path is wrong. Check the spelling, or `cd` into the folder that contains the file |
| The program seems stuck after you typed something | You forgot the period. Type `.` and press Enter |
| `Invalid option, please try again.` | Type a number from 1 to 9 followed by a period, e.g. `6.` |
| `Unknown location. Known locations: [...]` | Use one of the seven names shown in the message |
| `X is not a valid delivery location` | Colombo is the base, not a camp. Choose one of the six delivery points |
| `has already received its delivery` | That camp is already served. Use option 7 to reset |
| `TOUR DENIED` / `DELIVERY DENIED` | Not enough battery (or payload over 100 kg). Choose fewer camps or reset with option 7 |
| `No valid tour found` | Blocked roads have cut the drone off from a camp. Unblock a road with option 4 |
| Menu is gone after an error | Type `go.` again |

---

## 6. Project structure (for the code reader)

`COU4303_Group45.pl` is one file with these sections:

| Section | Purpose |
|---|---|
| Input helpers | Safe reading of user input (no crashes, case-insensitive names) |
| Map data | `road(CityA, CityB, Energy, Km)` – the single table for the whole map |
| A\* heuristic | `h/3` estimates and the `heuristic/3` wrapper |
| Search algorithms | `dfs/4`, `bfs/4`, `astar/4` |
| Tour planning | Multi-target tour and the best visiting order |
| Drone state | Battery level, current location, delivered camps |
| Menu and options | `go/0` and one predicate per menu option |
| Comparison | Option 8: DFS, BFS and A\* side by side |

**To change the map:** edit only the `road/4` facts, the `delivery_point/2` facts (camp weights) or the default `blocked/2` fact. If you change an energy value, run `?- check_heuristics.` to confirm the heuristic is still valid.

---

## 7. Team and submission

- **Course:** COU4303 – Artificial Intelligence, BSc (IT)
- **Group:** 45
