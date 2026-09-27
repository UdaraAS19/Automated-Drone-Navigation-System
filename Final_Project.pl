% Flight network: edge/3 stores raw energy cost; road_km/3 stores distance.

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

% Estimated remaining energy cost for A*.
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

% Approximate road distance in kilometres, separate from energy cost.
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

% path_distance_km(+Path, -Kilometres)
path_distance_km([_], 0).
path_distance_km([A, B | Rest], Km) :-
    (   road_km(A, B, Leg) ; road_km(B, A, Leg) ), !,
    path_distance_km([B | Rest], RestKm),
    Km is Leg + RestKm.

% Raw energy cost covered by a full battery.
full_charge_range(500).

% battery_percent(+RawCost, -Percent)
% SImplified battery calculation
battery_percent(Raw, Pct) :-
    Pct is (Raw * 100) // 500.

% blocked/2 stores no-fly zones.
:- dynamic(blocked/2).
blocked('Avissawella', 'Kandy'). % Bad weather pocket

% Relief camps and package weights in kilograms.

delivery_point('Colombo', 25).
delivery_point('Hambantota', 40).
delivery_point('Badulla', 30).

% A move is valid in either direction unless the airspace is blocked.

valid_move(Current, Next, Cost) :-
    (edge(Current, Next, Cost) ; edge(Next, Current, Cost)),
    \+ blocked(Current, Next),
    \+ blocked(Next, Current).

% Search algorithms return a route and its raw energy cost.

dfs(Start, Target, Path, Cost) :-
    dfs_helper(Start, Target, [Start], RevPath, Cost),
    reverse(RevPath, Path).

dfs_helper(Target, Target, Visited, Visited, 0).
dfs_helper(Current, Target, Visited, FinalPath, TotalCost) :-
    valid_move(Current, Next, StepCost),
    \+ member(Next, Visited),
    dfs_helper(Next, Target, [Next|Visited], FinalPath, RestCost),
    TotalCost is StepCost + RestCost.

% Breadth-first search minimizes the number of legs.
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

% A* search prioritizes routes by estimated total energy cost.
astar(Start, Target, Path, Cost) :-
    h(Start, Target, H),
    astar_search([[H,0,[Start]]], Target, RevPath, Cost),
    reverse(RevPath, Path).

astar_search([[_,Cost,[Target|Rest]]|_], Target, [Target|Rest], Cost).
astar_search([[F,G,[Current|Rest]]|Others], Target, Path, Cost):-
	findall([F2, G2,[Next,Current|Rest]],
		(valid_move(Current, Next, StepCost),
		\+ member(Next,[Current|Rest]),
		G2 is G + StepCost,
		h(Next,Target,H),
		F2 is G2 + H),
		Children),
	append(Others, Children, All),
	sort(All,SortedQueue),
	astar_search(SortedQueue,Target,Path,Cost).

% Helper to dynamically select the algorithm
run_algo(dfs, S, T, P, C) :- dfs(S, T, P, C).
run_algo(bfs, S, T, P, C) :- bfs(S, T, P, C).
run_algo(astar, S, T, P, C) :- astar(S, T, P, C).

% Visit each target in order, then return to the start.

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

% Drone state is maintained for the current session.

max_battery(100).

:- dynamic(current_location/1).
:- dynamic(battery_level/1).
:- dynamic(delivered/1).   % delivered(Location): relief camps already served this session

% Reset the position and battery while keeping delivery history.
reset_drone :-
    max_battery(Max),
    retractall(current_location(_)),
    retractall(battery_level(_)),
    assertz(current_location('Ratnapura')),
    assertz(battery_level(Max)).

% Start a new session and clear delivery history.
init_drone :-
    retractall(delivered(_)),
    reset_drone.

available_location(Loc, Weight) :-
    delivery_point(Loc, Weight),
    \+ delivered(Loc).

% Interactive delivery menu.

go :-
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
    write('3. Block a road'), nl,
    write('4. Unblock a road'), nl,
    write('5. Show blocked roads'), nl,
    write('6. Plan a Multi-Target Tour'), nl,
    write('7. Reset drone (return to base, refill battery)'), nl,
    write('8. Exit'), nl,
    write('Select an option (end with a period, e.g. 1.): '),
    read(Choice),
    handle_choice(Choice).

handle_choice(1) :- !, view_delivery_details, menu_loop.
handle_choice(2) :- !, deliver_to_location, menu_loop.
handle_choice(3) :- !, block_road, menu_loop.
handle_choice(4) :- !, unblock_road, menu_loop.
handle_choice(5) :- !, show_blocked, menu_loop.
handle_choice(6) :- !, plan_tour_menu, menu_loop.
handle_choice(7) :- !, reset_drone, nl, write('Drone reset: at Ratnapura, battery 100%.'), nl, menu_loop.
handle_choice(8) :- !, nl, write('Exiting Drone Relief Delivery System. Safe travels!'), nl.
handle_choice(end_of_file) :- !, nl, write('Exiting Drone Relief Delivery System.'), nl.
handle_choice(_) :- nl, write('Invalid option, please try again.'), nl, menu_loop.

% --- Multi-Target Tour Menu Logic ---
plan_tour_menu :-
    current_location(Start),
    nl, write('--- Multi-Target Tour Planning ---'), nl,
    write('Enter targets separated by commas and end with a period (e.g., \'Colombo\', \'Hambantota\'.): '),
    read_targets(Targets),
    (   Targets == [] -> nl, write('No targets specified.'), nl
    ;   nl, write('--- Comparing Algorithms for the Tour ---'), nl,
        % Compare DFS
        (   plan_tour(Start, Targets, dfs, FullPathDfs, TotalCostDfs) ->
            battery_percent(TotalCostDfs, PctDfs),
            path_distance_km(FullPathDfs, KmDfs),
            format('Algorithm: dfs   | Distance: ~w km | Battery Cost: ~w% | Full Route: ~w~n', [KmDfs, PctDfs, FullPathDfs])
        ;   write('Algorithm: dfs   | No valid tour found.'), nl
        ),
        % Compare BFS
        (   plan_tour(Start, Targets, bfs, FullPathBfs, TotalCostBfs) ->
            battery_percent(TotalCostBfs, PctBfs),
            path_distance_km(FullPathBfs, KmBfs),
            format('Algorithm: bfs   | Distance: ~w km | Battery Cost: ~w% | Full Route: ~w~n', [KmBfs, PctBfs, FullPathBfs])
        ;   write('Algorithm: bfs   | No valid tour found.'), nl
        ),
        % Compare A*
        (   plan_tour(Start, Targets, astar, FullPathAstar, TotalCostAstar) ->
            battery_percent(TotalCostAstar, PctAstar),
            path_distance_km(FullPathAstar, KmAstar),
            format('Algorithm: astar | Distance: ~w km | Battery Cost: ~w% | Full Route: ~w~n', [KmAstar, PctAstar, FullPathAstar])
        ;   write('Algorithm: astar | No valid tour found.'), nl
        )
    ).

% Helper to successfully read a comma-separated tuple ending with a period.
read_targets(Targets) :-
    read(Term),
    tuple_to_list(Term, Targets).

tuple_to_list((A, B), [A | Rest]) :-
    !, tuple_to_list(B, Rest).
tuple_to_list(A, [A]) :- A \== end_of_file, A \== ''.
tuple_to_list(_, []).


% Block or unblock a known road.
block_road :-
    nl, write('Enter Road to Block (example: \'Avissawella\'. \'Kandy\'.): '),
    read(A), read(B),
    (   known_road(A, B)
    ->  (   blocked(A, B) ; blocked(B, A)
        ->  nl, write('That road is already blocked.'), nl
        ;   assertz(blocked(A, B)),
            nl, write('Road blocked: '), write(A-B), nl
        )
    ;   nl, write('Unknown road. No road was blocked.'), nl
    ).

known_road(A, B) :- edge(A, B, _).
known_road(A, B) :- edge(B, A, _).

unblock_road :-
    nl, write('Enter Road to Unblock (example: \'Avissawella\'. \'Kandy\'.): '),
    read(A), read(B),
    (   known_road(A, B),
        (blocked(A, B) ; blocked(B, A))
    ->  retractall(blocked(A, B)),
        retractall(blocked(B, A)),
        write('Unblocked: '), write(A-B), nl
    ;   nl, write('No such blocked road found.'), nl
    ).

show_blocked :-
    nl, write('Currently blocked roads: '), nl, nl,
    (blocked(_, _) -> list_blocked ; write('None'), nl).

list_blocked :- forall(blocked(A,B),(write(A-B), nl)).

% Display route distance, battery cost, and package weight.
show_delivery_details(Loc, Path, Cost, Weight) :-
    path_distance_km(Path, Km),
    battery_percent(Cost, Pct),
    format('Location: ~w~n', [Loc]),
    format('  Distance: ~w km~n', [Km]),
    format('  Route: ~w~n', [Path]),
    format('  Battery Cost: ~w%~n', [Pct]),
    format('  Package Weight: ~wkg~n', [Weight]).

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

review_and_confirm(Current, Target, Path, Cost, Weight) :-
    battery_percent(Cost, Pct),
    battery_level(Bat),
    nl, write('--- Delivery Details ---'), nl,
    show_delivery_details(Target, Path, Cost, Weight),
    format('  Battery Available: ~w%~n', [Bat]),
    (   Pct =< Bat
    ->  Remaining is Bat - Pct,
        format('  Battery After Delivery: ~w%~n', [Remaining]),
        nl,
        (   ask_confirmation
        ->  execute_delivery(Current, Target, Path, Pct, Weight)
        ;   nl, write('Delivery cancelled. No battery used.'), nl
        )
    ;   nl, write('*** DELIVERY DENIED ***'), nl,
        format('Insufficient battery for this trip. Required: ~w%, Available: ~w%~n', [Pct, Bat])
    ).

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

execute_delivery(Current, Target, Path, Pct, Weight) :-
    battery_level(Bat),
    NewBat is Bat - Pct,
    retract(battery_level(Bat)),
    assertz(battery_level(NewBat)),
    retract(current_location(Current)),
    assertz(current_location(Target)),
    assertz(delivered(Target)),   % remove this camp from future listings
    nl, write('*** DELIVERY APPROVED ***'), nl,
    format('Route Taken: ~w~n', [Path]),
    format('Battery Used: ~w%   |   Battery Remaining: ~w%~n', [Pct, NewBat]),
    format('Delivered ~wkg of supplies to ~w.~n', [Weight, Target]).