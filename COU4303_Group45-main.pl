% ==============================================================================
% 1. KNOWLEDGE BASE (KB.pl)
% Represents topological maps, waypoints, and edge costs (battery %).
% ==============================================================================

% edge(Node1, Node2, BatteryCost).
edge(base, wp1, 10).
edge(wp1, camp_alpha, 15).
edge(base, wp2, 20).
edge(wp2, camp_alpha, 10).
edge(camp_alpha, wp3, 12).
edge(wp3, camp_beta, 18).
edge(camp_beta, base, 25).
edge(camp_alpha, base, 30).
edge(wp1, camp_beta, 35).

% Heuristic h(n) for A*: Direct line-of-sight distance to target[cite: 4].
% h(CurrentNode, TargetNode, EstimatedCost).
h(wp1, camp_alpha, 10).
h(wp2, camp_alpha, 8).
h(wp3, camp_beta, 15).
h(camp_alpha, camp_beta, 25).
h(camp_beta, base, 20).
h(camp_alpha, base, 25).
h(X, X, 0). % Distance to itself is 0
h(_, _, 10). % Fallback heuristic for unlisted direct pairs

% Dynamic Constraints: blocked(Node1, Node2) represents No-fly zones.
blocked(base, wp2). % Bad weather pocket

% ==============================================================================
% 1b. RELIEF CAMPS / DELIVERY POINTS
% These are the locations where supplies (goods) are received by default.
% delivery_point(Location, PackageWeightKg).
% ==============================================================================

delivery_point(camp_alpha, 25).
delivery_point(camp_beta, 40).

% ==============================================================================
% 2. CONSTRAINT & SAFETY ENGINE
% Validates moves based on bidirectional edges and blocked airspace[cite: 4].
% ==============================================================================

valid_move(Current, Next, Cost) :-
    (edge(Current, Next, Cost) ; edge(Next, Current, Cost)),
    \+ blocked(Current, Next),
    \+ blocked(Next, Current).

% ==============================================================================
% 3. CORE SEARCH ALGORITHMS (BFS, DFS, A*)
% Finds a route between two locations[cite: 1].
% ==============================================================================

% --- DFS (Path) ---
dfs(Start, Target, Path, Cost) :-
    dfs_helper(Start, Target, [Start], RevPath, Cost),
    reverse(RevPath, Path).

dfs_helper(Target, Target, Visited, Visited, 0).
dfs_helper(Current, Target, Visited, FinalPath, TotalCost) :-
    valid_move(Current, Next, StepCost),
    \+ member(Next, Visited),
    dfs_helper(Next, Target, [Next|Visited], FinalPath, RestCost),
    TotalCost is StepCost + RestCost.

% --- BFS (Unweighted Path/Shortest Hops) ---
bfs(Start, Target, Path, Cost) :-
    bfs_queue([ [[Start], 0] ], Target, RevPath, Cost),
    reverse(RevPath, Path).

bfs_queue([ [[Target|Rest], Cost] | _ ], Target, [Target|Rest], Cost).
bfs_queue([ [[Current|Rest], Cost] | QueueTail ], Target, FinalPath, FinalCost) :-
    findall(
        [[Next, Current|Rest], NewCost],
        (valid_move(Current, Next, StepCost), \+ member(Next, [Current|Rest]), NewCost is Cost + StepCost),
        Children
    ),
    append(QueueTail, Children, NewQueue),
    bfs_queue(NewQueue, Target, FinalPath, FinalCost).

% --- A* (Optimal Cost) ---
astar(Start, Target, Path, Cost) :-
    h(Start, Target, H),
    astar_queue([H-[[Start], 0]], Target, RevPath, Cost),
    reverse(RevPath, Path).

astar_queue([_-[[Target|Rest], Cost] | _], Target, [Target|Rest], Cost).
astar_queue([_-[[Current|Rest], Cost] | QueueTail], Target, FinalPath, FinalCost) :-
    findall(
        F-[[Next, Current|Rest], NewCost],
        (
            valid_move(Current, Next, StepCost),
            \+ member(Next, [Current|Rest]),
            NewCost is Cost + StepCost,
            h(Next, Target, H),
            F is NewCost + H
        ),
        Children
    ),
    append(QueueTail, Children, Unsorted),
    keysort(Unsorted, SortedQueue), % Priority queue sorted by F-value
    astar_queue(SortedQueue, Target, FinalPath, FinalCost).

% Helper to dynamically select the algorithm
run_algo(dfs, S, T, P, C) :- dfs(S, T, P, C).
run_algo(bfs, S, T, P, C) :- bfs(S, T, P, C).
run_algo(astar, S, T, P, C) :- astar(S, T, P, C).

% ==============================================================================
% 4. MULTI-TARGET TOUR PLANNER MODULE
% Base -> T1 -> T2 -> ... -> Base[cite: 4].
% ==============================================================================

plan_tour(Start, Targets, Algo, FullPath, TotalCost) :-
    append(Targets, [Start], FullRoute),
    plan_legs(Start, FullRoute, Algo, FullPath, TotalCost).

plan_legs(_, [], _, [], 0).
plan_legs(Current, [NextTarget|Rest], Algo, Path, TotalCost) :-
    run_algo(Algo, Current, NextTarget, LegPath, LegCost),
    plan_legs(NextTarget, Rest, Algo, RestPath, RestCost),
    TotalCost is LegCost + RestCost,
    combine_paths(LegPath, RestPath, Path).

% Combines paths by dropping the duplicated waypoint connecting two legs
combine_paths(P1, [], P1) :- !.
combine_paths(P1, [_|T2], Combined) :- append(P1, T2, Combined).

% ==============================================================================
% 5. METRICS & COMPARISON ENGINE
% Compares path length and battery used for presentation metrics[cite: 4].
% ==============================================================================

compare_algorithms(Start, Target) :-
    nl, write('--- Algorithm Performance Comparison ---'), nl,
    compare_run(bfs, Start, Target),
    compare_run(dfs, Start, Target),
    compare_run(astar, Start, Target).

compare_run(Algo, Start, Target) :-
    ( run_algo(Algo, Start, Target, Path, Cost) ->
        length(Path, Hops), NodeCount is Hops - 1,
        format('Algorithm: ~w | Hops: ~w | Battery Cost: ~w% | Path: ~w~n', [Algo, NodeCount, Cost, Path])
    ;
        format('Algorithm: ~w | No valid path found.~n', [Algo])
    ).

% ==============================================================================
% 6. DRONE STATE (Live position + battery, tracked across the whole session)
% ==============================================================================

:- dynamic(current_location/1).
:- dynamic(battery_level/1).

% Reset the drone to its starting state: parked at base, battery full.
init_drone :-
    retractall(current_location(_)),
    retractall(battery_level(_)),
    assertz(current_location(base)),
    assertz(battery_level(100)).

% ==============================================================================
% 7. MAIN MENU & DELIVERY MANAGEMENT SYSTEM
% - Lists relief-camp delivery points with distance / route / battery cost / weight.
% - Lets the user pick a location and press "Deliver".
% - Before every trip (including trips after the first), checks that enough
%   battery remains for that leg's cost; the trip is only permitted if it does.
% ==============================================================================

% Entry point: run this to start the program -> ?- main_menu.
main_menu :-
    init_drone,
    menu_loop.

menu_loop :-
    nl, write('=============================================='), nl,
    write('        DRONE RELIEF DELIVERY - MAIN MENU'), nl,
    write('=============================================='), nl,
    current_location(Loc), battery_level(Bat),
    format('Current Location: ~w   |   Battery Remaining: ~w%~n', [Loc, Bat]),
    write('----------------------------------------------'), nl,
    write('1. View delivery locations (distance/route/battery cost/weight)'), nl,
    write('2. Deliver to a selected location'), nl,
    write('3. Compare algorithms for a custom route'), nl,
    write('4. Reset drone (return to base, refill battery)'), nl,
    write('5. Exit'), nl,
    write('Select an option (end with a period, e.g. 1.): '),
    read(Choice),
    handle_choice(Choice).

handle_choice(1) :- !, view_delivery_details, menu_loop.
handle_choice(2) :- !, deliver_to_location, menu_loop.
handle_choice(3) :- !, compare_algorithms_menu, menu_loop.
handle_choice(4) :- !, init_drone, nl, write('Drone reset: at base, battery 100%.'), nl, menu_loop.
handle_choice(5) :- !, nl, write('Exiting Drone Relief Delivery System. Safe travels!'), nl.
handle_choice(_) :- nl, write('Invalid option, please try again.'), nl, menu_loop.

% --- Option 1: show every relief camp's distance, route, battery cost, weight ---
view_delivery_details :-
    current_location(Current),
    nl, write('--- Delivery Locations (goods to be delivered) ---'), nl,
    format('(Calculated from current location: ~w)~n', [Current]),
    forall(
        delivery_point(Loc, Weight),
        (   astar(Current, Loc, Path, Cost)
        ->  length(Path, Hops), Distance is Hops - 1,
            format('Location: ~w~n', [Loc]),
            format('  Distance: ~w hop(s)~n', [Distance]),
            format('  Route: ~w~n', [Path]),
            format('  Battery Cost: ~w%~n', [Cost]),
            format('  Package Weight: ~wkg~n', [Weight])
        ;   format('Location: ~w | No valid path found from ~w.~n', [Loc, Current])
        )
    ).

% --- Option 2: pick a location, then deliver (battery-checked) ---
deliver_to_location :-
    nl, write('Available locations: '),
    findall(L, delivery_point(L, _), Locations),
    write(Locations), nl,
    write('Enter destination location (end with a period, e.g. camp_alpha.): '),
    read(Target),
    (   delivery_point(Target, Weight)
    ->  current_location(Current),
        (   astar(Current, Target, Path, Cost)
        ->  attempt_delivery(Current, Target, Path, Cost, Weight)
        ;   format('No valid path from ~w to ~w. Delivery not possible.~n', [Current, Target])
        )
    ;   nl, write('Invalid delivery location.'), nl
    ).

% Checks remaining battery before permitting the trip (applies to every trip,
% including the second and later ones after the first delivery).
attempt_delivery(Current, Target, Path, Cost, Weight) :-
    battery_level(Bat),
    (   Cost =< Bat
    ->  NewBat is Bat - Cost,
        retract(battery_level(Bat)),
        assertz(battery_level(NewBat)),
        retract(current_location(Current)),
        assertz(current_location(Target)),
        nl, write('*** DELIVERY APPROVED ***'), nl,
        format('Route Taken: ~w~n', [Path]),
        format('Battery Used: ~w%   |   Battery Remaining: ~w%~n', [Cost, NewBat]),
        format('Delivered ~wkg of supplies to ~w.~n', [Weight, Target])
    ;   nl, write('*** DELIVERY DENIED ***'), nl,
        format('Insufficient battery for this trip. Required: ~w%, Available: ~w%~n', [Cost, Bat])
    ).

% --- Option 3: free-form algorithm comparison between any two nodes ---
compare_algorithms_menu :-
    nl, write('Enter start location (end with a period): '), read(Start),
    write('Enter target location (end with a period): '), read(Target),
    compare_algorithms(Start, Target).

% Auto-start the menu when the file is consulted/run.
:- initialization(main_menu).

