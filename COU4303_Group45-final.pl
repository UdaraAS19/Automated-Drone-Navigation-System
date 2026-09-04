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

% Optional coordinates for a simple UI/map rendering and distance calculations
% coord(Node, X, Y).
coord(base, 0, 0).
coord(a, 1, 1).
coord(b, 2, 0).
coord(c, 2, 2).
coord(d, 3, 1).
coord(target1, 4, 2).
coord(target2, 5, 1).

% Relief camp facts and payload weights (kg)
relief_camp(target1).
relief_camp(target2).
weight(target1, 50). % 50 kg of supplies
weight(target2, 30). % 30 kg of supplies

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
    format('\n--- Mission Execution: ~w ---~n', [Algorithm]),
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

% --- 6. USER INTERFACE / MAIN MENU ---
% Provides a simple text-based UI for exploring relief camps and routes relative to the main warehouse (base).

% Euclidean distance between two nodes using their coordinates
euclidean_distance(A, B, D) :-
    coord(A, X1, Y1),
    coord(B, X2, Y2),
    DX is X2 - X1,
    DY is Y2 - Y1,
    D is sqrt(DX*DX + DY*DY).

% Display a simple textual map: list of camps with relative coordinates and straight-line distance from base
display_map :-
    format('\n--- Relief Camps Map (relative to base at (0,0)) ---~n'),
    coord(base, BX, BY),
    findall(Name-X-Y-D, (relief_camp(Name), coord(Name, X, Y), euclidean_distance(base, Name, D)), Entries),
    (Entries = [] -> format('No relief camps defined.\n') ; display_map_entries(Entries)),
    format('-----------------------------------------------------~n').

display_map_entries([]).
display_map_entries([Name-X-Y-D|Rest]) :-
    format('Camp: ~w at (~w,~w)  |  Straight-line distance from base: ~2f~n', [Name, X, Y, D]),
    display_map_entries(Rest).

% List camps with computed travel cost (using A*) and payload weight
list_camps :-
    format('\n--- Available Relief Camps ---~n'),
    findall(C, relief_camp(C), Camps),
    (Camps = [] -> format('No camps available.\n') ; list_camps_indexed(Camps, 1)),
    format('-----------------------------~n').

list_camps_indexed([], _).
list_camps_indexed([C|Rest], N) :-
    ( find_path(astar, base, C, _Path, Cost) -> true ; Cost = unknown ),
    ( weight(C, W) -> true ; W = unknown ),
    format('~w) ~w  |  Travel cost (battery estimate): ~w  |  Weight: ~w kg~n', [N, C, Cost, W]),
    N1 is N + 1,
    list_camps_indexed(Rest, N1).

% Map an index to a relief camp name
select_target_by_index(Index, Target) :-
    findall(C, relief_camp(C), Camps),
    nth1(Index, Camps, Target).

% Show details for a particular camp: route, battery cost and straight-line distance and payload
show_camp_details(Camp) :-
    ( find_path(astar, base, Camp, Path, Cost) ->
        format('\n--- Camp Details: ~w ---~n', [Camp]),
        format('Planned route (A*): ~w~n', [Path]),
        format('Battery cost (path): ~w~n', [Cost])
    ;
        format('No viable path to ~w (blocked or undefined).~n', [Camp])
    ),
    ( euclidean_distance(base, Camp, EucDist) -> format('Straight-line distance: ~2f~n', [EucDist]) ; true ),
    ( weight(Camp, W) -> format('Payload weight: ~w kg~n', [W]) ; format('Payload weight: unknown~n')), 
    format('-------------------------------~n').

% Interactive camp selection menu
camp_menu :-
    repeat,
    format('\nEnter camp number to view details, or B to go back: '),
    read_line_to_string(user_input, Input),
    ( Input = "B" ; Input = "b" -> ! ;
      ( number_string(Idx, Input) ->
            ( select_target_by_index(Idx, Target) -> show_camp_details(Target) ; format('Invalid index.~n') ),
            fail
      ; format('Invalid input. Enter a number or B to go back.~n'), fail
      )
    ).

% Main menu loop
main_menu :-
    format('\n=== Autonomous Drone Navigator - MAIN MENU ===~n'),
    repeat,
    format('\nOptions:\n'),
    format('1) Show map of relief camps~n'),
    format('2) List camps and view details~n'),
    format('3) Compare search algorithms (demo)~n'),
    format('4) Exit\n'),
    format('Enter choice (1-4): '),
    read_line_to_string(user_input, Choice),
    ( Choice = "1" -> display_map, fail ;
      Choice = "2" -> list_camps, camp_menu, fail ;
      Choice = "3" -> compare_algorithms(base, [target1, target2], 120), fail ;
      Choice = "4" -> format('Exiting main menu.\n'), ! ;
      format('Invalid option. Please choose 1-4.~n'), fail
    ).

% Allow quick start by running main_menu/0 when file is loaded in interactive session
:- multifile user:message_hook/3.
user:message_hook(_Term, _Kind, _Lines) :- false.

% End of file
