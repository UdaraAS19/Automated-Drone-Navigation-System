% ==============================================================================
% AUTONOMOUS DRONE NAVIGATOR (Disaster Zone Medical Delivery)
% Course: COU4303 - Artificial Intelligence
% Component: Mini Project Source Code
% ==============================================================================

% --- 1. KNOWLEDGE BASE & MAP MODELING ---
% connected(Node1, Node2, BatteryCost).
connected(base, a, 10).
connected(base, b, 25).
connected(a, c, 15).
connected(b, c, 10).
connected(b, d, 20).
connected(c, d, 5).
connected(c, target1, 15).
connected(d, target2, 10).
connected(target1, target2, 20).

% Make paths bidirectional
edge(X, Y, Cost) :- connected(X, Y, Cost).
edge(X, Y, Cost) :- connected(Y, X, Cost).

% Straight-line distance heuristics for A* search (Simplified)
% h(CurrentNode, TargetNode, EstimatedCost).
h(a, target1, 10).
h(b, target1, 15).
h(c, target1, 5).
h(X, X, 0).
h(_, _, 5). % Default fallback heuristic

% --- 2. CONSTRAINT & SAFETY ENGINE ---
% Represents realistic constraints such as blocked roads / air-restrictions
restricted(b). % Example: Node 'b' is a restricted airspace due to storm

% Check if a node is safe for flight
valid_node(Node) :- \+ restricted(Node).

% --- 3. CORE SEARCH ALGORITHMS ---

% --- Depth First Search (DFS) ---
dfs(Start, Target, Path, Cost) :-
    dfs_util(Start, Target, [Start], RevPath, Cost),
    reverse(RevPath, Path).

dfs_util(Target, Target, Visited, Visited, 0).
dfs_util(Current, Target, Visited, Path, TotalCost) :-
    edge(Current, Next, StepCost),
    valid_node(Next),
    \+ member(Next, Visited),
    dfs_util(Next, Target, [Next|Visited], Path, RestCost),
    TotalCost is StepCost + RestCost.

% --- Breadth First Search (BFS) ---
bfs(Start, Target, Path, Cost) :-
    bfs_queue([[Start]], Target, RevPath),
    reverse(RevPath, Path),
    path_cost(Path, Cost).

bfs_queue([[Target|Rest]|_], Target, [Target|Rest]).
bfs_queue([[Current|Rest]|Queue], Target, Path) :-
    findall([Next, Current|Rest],
            (edge(Current, Next, _), valid_node(Next), \+ member(Next, [Current|Rest])),
            NextPaths),
    append(Queue, NextPaths, NewQueue),
    bfs_queue(NewQueue, Target, Path).

path_cost([_], 0).
path_cost([A, B|Rest], Total) :-
    edge(A, B, Cost),
    path_cost([B|Rest], RestCost),
    Total is Cost + RestCost.

% --- A* Search (Optimal Cost) ---
% Uses F = G + H structure via F-node(Current, Path, G) for SWI-Prolog keysort
a_star(Start, Target, Path, Cost) :-
    h(Start, Target, H),
    a_star_queue([H-node(Start, [Start], 0)], Target, RevPath, Cost),
    reverse(RevPath, Path).

a_star_queue([_-node(Target, Path, Cost)|_], Target, Path, Cost).
a_star_queue([_-node(Current, Path, G)|RestQueue], Target, FinalPath, FinalCost) :-
    findall(NewF-node(Next, [Next|Path], NewG),
            (edge(Current, Next, StepCost),
             valid_node(Next),
             \+ member(Next, Path),
             NewG is G + StepCost,
             h(Next, Target, H),
             NewF is NewG + H),
            Children),
    append(RestQueue, Children, UnsortedQueue),
    keysort(UnsortedQueue, SortedQueue), % Sort priority queue by F value
    a_star_queue(SortedQueue, Target, FinalPath, FinalCost).

% --- 4. MULTI-TARGET TOUR PLANNER MODULE ---
% Plans the route through multiple targets and calculates accumulated battery cost
plan_tour(_, Current, [], Current, [], 0).
plan_tour(Algorithm, Current, [NextTarget|RestTargets], FinalBase, TotalPath, TotalCost) :-
    find_path(Algorithm, Current, NextTarget, PathSeg, CostSeg),
    plan_tour(Algorithm, NextTarget, RestTargets, FinalBase, RestPathSeg, RestCost),
    append_paths(PathSeg, RestPathSeg, TotalPath),
    TotalCost is CostSeg + RestCost.

% Algorithm dispatcher
find_path(dfs, S, T, P, C) :- dfs(S, T, P, C).
find_path(bfs, S, T, P, C) :- bfs(S, T, P, C).
find_path(astar, S, T, P, C) :- a_star(S, T, P, C).

% Joins paths seamlessly without duplicating the overlapping waypoint
append_paths(P1, [_|P2], P) :- append(P1, P2, P).
append_paths(P1, [], P1).

% Runs the specific mission sequence
execute_mission(Algorithm, StartBase, Targets, BatteryLimit) :-
    format('~n--- Mission Execution: ~w ---~n', [Algorithm]),
    plan_tour(Algorithm, StartBase, Targets, LastStop, OutboundPath, OutboundCost),
    find_path(Algorithm, LastStop, StartBase, ReturnPath, ReturnCost),
    append_paths(OutboundPath, ReturnPath, FullRoute),
    TotalBatteryUsed is OutboundCost + ReturnCost,
    format('Route Planned: ~w~n', [FullRoute]),
    format('Cost (Battery Used): ~w / ~w limit~n', [TotalBatteryUsed, BatteryLimit]),
    (TotalBatteryUsed =< BatteryLimit ->
        format('Status: SUCCESS. Drone safely delivered supplies and returned.~n')
    ;
        format('Status: FAILED. Insufficient battery limits.~n')
    ).

% --- 5. METRICS & COMPARISON ENGINE ---
% Query this to compare all algorithms: "?- compare_algorithms(base, [target1, target2], 120)."
compare_algorithms(StartBase, Targets, BatteryLimit) :-
    execute_mission(dfs, StartBase, Targets, BatteryLimit),
    execute_mission(bfs, StartBase, Targets, BatteryLimit),
    execute_mission(astar, StartBase, Targets, BatteryLimit).
