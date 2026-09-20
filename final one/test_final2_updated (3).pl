
% ==============================================================================
% 1. KNOWLEDGE BASE (KB.pl)
% Represents topological maps, waypoints, and edge costs (battery %).
% ==============================================================================

% edge(Node1, Node2, EnergyCost).  Costs are raw energy units; they are converted
% to battery-% by battery_percent/2 below (scaled to the drone's battery capacity).
edge('Ratnapura', 'Kalutara', 55).
edge('Ratnapura', 'Balangoda', 42).
edge('Ratnapura', 'Avissawella', 45).
edge('Kalutara', 'Galle', 70).
edge('Kalutara', 'Colombo', 43).
edge('Avissawella', 'Colombo', 36).
edge('Avissawella', 'Kandy', 65).
edge('Balangoda', 'Badulla', 68).
edge('Galle', 'Matara', 35).
edge('Matara', 'Hambantota', 140).   % raised so Hambantota costs 60% from Ratnapura
edge('Badulla', 'Hambantota', 190).   % raised so Hambantota costs 60% from Ratnapura
edge('Kandy', 'Badulla', 105).

% Heuristic h(n) for A*: Direct line-of-sight distance to target.
% h(CurrentNode, TargetNode, EstimatedCost).
h('Ratnapura', 'Hambantota', 100).
h('Balangoda', 'Hambantota', 90).
h('Badulla', 'Hambantota', 100).
h('Galle', 'Hambantota', 105).
h('Matara', 'Hambantota', 70).
h('Ratnapura', 'Colombo', 50).
h('Kalutara', 'Colombo', 40).
h('Avissawella', 'Colombo', 30).
h(X, X, 0). % Distance to itself is 0
h(_, _, 20). % Fallback heuristic for unlisted direct pairs

% Road distances in kilometres between connected towns (approximate driving distances).
% road_km(Node1, Node2, Km).  Used for the "Distance" shown to the user;
% battery cost above is a separate energy figure.
road_km('Ratnapura', 'Kalutara', 80).
road_km('Ratnapura', 'Balangoda', 47).
road_km('Ratnapura', 'Avissawella', 51).
road_km('Kalutara', 'Galle', 74).
road_km('Kalutara', 'Colombo', 42).
road_km('Avissawella', 'Colombo', 40).
road_km('Avissawella', 'Kandy', 88).
road_km('Balangoda', 'Badulla', 90).
road_km('Galle', 'Matara', 45).
road_km('Matara', 'Hambantota', 85).
road_km('Badulla', 'Hambantota', 165).
road_km('Kandy', 'Badulla', 130).

% path_distance_km(Path, Km): total road distance along a list of waypoints.
path_distance_km([_], 0).
path_distance_km([A, B | Rest], Km) :-
    (   road_km(A, B, Leg) ; road_km(B, A, Leg) ), !,
    path_distance_km([B | Rest], RestKm),
    Km is Leg + RestKm.

% Battery scaling: a full charge (max_battery/1, i.e. 100%) can cover this many raw
% cost units. Raw route costs are converted into a % of battery capacity, so
% e.g. a 220-unit route uses 220 * 100 / 500 = 44% of the battery.
full_charge_range(500).

% battery_percent(RawCost, BatteryPercent): raw cost -> % of battery (1 decimal).
battery_percent(Raw, Pct) :-
    max_battery(Max),
    full_charge_range(Range),
    Pct is round(Raw * Max * 10 / Range) / 10.

% Dynamic Constraints: blocked(Node1, Node2) represents No-fly zones.
blocked('Avissawella', 'Kandy'). % Bad weather pocket

% ==============================================================================
% 1b. RELIEF CAMPS / DELIVERY POINTS
% These are the locations where supplies (goods) are received by default.
% delivery_point(Location, PackageWeightKg).
% ==============================================================================

delivery_point('Colombo', 25).
delivery_point('Hambantota', 40).
delivery_point('Badulla', 30).

% ==============================================================================
% 2. CONSTRAINT & SAFETY ENGINE
% Validates moves based on bidirectional edges and blocked airspace.
% ==============================================================================

valid_move(Current, Next, Cost) :-
    (edge(Current, Next, Cost) ; edge(Next, Current, Cost)),
    \+ blocked(Current, Next),
    \+ blocked(Next, Current).

% ==============================================================================
% 3. CORE SEARCH ALGORITHMS (BFS, DFS, A*)
% Finds a route between two locations.
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
% ==============================================================================

compare_algorithms(Start, Target) :-
    nl, write('--- Algorithm Performance Comparison ---'), nl,
    compare_run(bfs, Start, Target),
    compare_run(dfs, Start, Target),
    compare_run(astar, Start, Target).

compare_run(Algo, Start, Target) :-
    ( run_algo(Algo, Start, Target, Path, Cost) ->
        length(Path, Hops), NodeCount is Hops - 1,
        battery_percent(Cost, Pct),
        path_distance_km(Path, Km),
        format('Algorithm: ~w | Hops: ~w | Distance: ~w km | Battery Cost: ~w% | Path: ~w~n', [Algo, NodeCount, Km, Pct, Path])
    ;
        format('Algorithm: ~w | No valid path found.~n', [Algo])
    ).

% ==============================================================================
% 6. DRONE STATE (Live position + battery, tracked across the whole session)
% ==============================================================================

% Maximum (and starting) battery capacity: the drone always operates on a 0-100% scale.
max_battery(100).

:- dynamic(current_location/1).
:- dynamic(battery_level/1).
:- dynamic(delivered/1).   % delivered(Location): relief camps already served this session

% Return the drone to base ('Ratnapura') and refill the battery to 100%.
% Delivered locations are kept, so served camps stay hidden.
reset_drone :-
    max_battery(Max),
    retractall(current_location(_)),
    retractall(battery_level(_)),
    assertz(current_location('Ratnapura')),
    assertz(battery_level(Max)).

% Full start-up state: reset the drone AND forget all previous deliveries.
init_drone :-
    retractall(delivered(_)),
    reset_drone.

% A relief camp is still "available" until the drone has delivered to it.
available_location(Loc, Weight) :-
    delivery_point(Loc, Weight),
    \+ delivered(Loc).

% ==============================================================================
% 7. MAIN MENU & DELIVERY MANAGEMENT SYSTEM
% ==============================================================================

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
handle_choice(4) :- !, reset_drone, nl, write('Drone reset: at Ratnapura, battery 100%.'), nl, menu_loop.
handle_choice(5) :- !, nl, write('Exiting Drone Relief Delivery System. Safe travels!'), nl.
handle_choice(_) :- nl, write('Invalid option, please try again.'), nl, menu_loop.

% --- Shared display: location, distance, route, battery cost, package weight ---
% (Cost is the raw route cost; it is shown as a % of battery capacity.)
show_delivery_details(Loc, Path, Cost, Weight) :-
    path_distance_km(Path, Km),
    battery_percent(Cost, Pct),
    format('Location: ~w~n', [Loc]),
    format('  Distance: ~w km~n', [Km]),
    format('  Route: ~w~n', [Path]),
    format('  Battery Cost: ~w%~n', [Pct]),
    format('  Package Weight: ~wkg~n', [Weight]).

% --- Option 1: show every pending relief camp's distance, route, battery cost, weight ---
view_delivery_details :-
    current_location(Current),
    nl, write('--- Delivery Locations (goods to be delivered) ---'), nl,
    format('(Calculated from current location: ~w)~n', [Current]),
    (   \+ available_location(_, _)
    ->  write('All relief camps have been served. No pending deliveries.'), nl
    ;   forall(
            available_location(Loc, Weight),
            (   astar(Current, Loc, Path, Cost)
            ->  show_delivery_details(Loc, Path, Cost, Weight)
            ;   format('Location: ~w | No valid path found from ~w.~n', [Loc, Current])
            )
        )
    ).

% --- Option 2: pick a location, review the details, confirm, then deliver ---
deliver_to_location :-
    findall(L, available_location(L, _), Locations),
    (   Locations == []
    ->  nl, write('All relief camps have been served. Nothing left to deliver.'), nl
    ;   nl, write('Available locations: '), write(Locations), nl,
        write('Enter destination location (end with a period, e.g. \'Colombo\'.): '),
        read(Target),
        (   available_location(Target, Weight)
        ->  current_location(Current),
            (   astar(Current, Target, Path, Cost)
            ->  review_and_confirm(Current, Target, Path, Cost, Weight)
            ;   format('No valid path from ~w to ~w. Delivery not possible.~n', [Current, Target])
            )
        ;   delivered(Target)
        ->  nl, format('~w has already received its delivery.~n', [Target])
        ;   nl, write('Invalid delivery location.'), nl
        )
    ).

% Show the delivery details, check the battery, and only deliver once the user confirms.
review_and_confirm(Current, Target, Path, Cost, Weight) :-
    battery_percent(Cost, Pct),
    battery_level(Bat),
    nl, write('--- Delivery Details ---'), nl,
    show_delivery_details(Target, Path, Cost, Weight),
    format('  Battery Available: ~w%~n', [Bat]),
    (   Pct =< Bat
    ->  Remaining is round((Bat - Pct) * 10) / 10,
        format('  Battery After Delivery: ~w%~n', [Remaining]),
        nl,
        (   ask_confirmation
        ->  execute_delivery(Current, Target, Path, Pct, Weight)
        ;   nl, write('Delivery cancelled. No battery used.'), nl
        )
    ;   nl, write('*** DELIVERY DENIED ***'), nl,
        format('Insufficient battery for this trip. Required: ~w%, Available: ~w%~n', [Pct, Bat])
    ).

% Succeeds on yes/y, fails on no/n (or end of input); re-asks on anything else.
ask_confirmation :-
    write('Confirm this delivery? (yes. / no.): '),
    read(Answer),
    (   memberchk(Answer, [yes, y])
    ->  true
    ;   memberchk(Answer, [no, n, end_of_file])
    ->  fail
    ;   write('Please answer yes. or no.'), nl,
        ask_confirmation
    ).

% Carry out a confirmed delivery: use battery, move the drone, mark the camp as served.
execute_delivery(Current, Target, Path, Pct, Weight) :-
    battery_level(Bat),
    NewBat is round((Bat - Pct) * 10) / 10,
    retract(battery_level(Bat)),
    assertz(battery_level(NewBat)),
    retract(current_location(Current)),
    assertz(current_location(Target)),
    assertz(delivered(Target)),   % remove this camp from future listings
    nl, write('*** DELIVERY APPROVED ***'), nl,
    format('Route Taken: ~w~n', [Path]),
    format('Battery Used: ~w%   |   Battery Remaining: ~w%~n', [Pct, NewBat]),
    format('Delivered ~wkg of supplies to ~w.~n', [Weight, Target]).

% --- Option 3: free-form algorithm comparison between any two nodes ---
compare_algorithms_menu :-
    nl, write('Enter start location (end with a period): '), read(Start),
    write('Enter target location (end with a period): '), read(Target),
    compare_algorithms(Start, Target).

:- initialization(main_menu).
