%------------ AUTOMATED DRONE RELIEF DELIVERY SYSTEM ------------
% COU4303 Mini Project - route finding with DFS, BFS and A*.
%
% HOW TO RUN
%   ?- [file_name].      (consult the file)
%   ?- go.               interactive menu
%   ?- demo.             automatic demonstration (no typing needed)
%   ?- check_heuristics. proves every A* heuristic value is admissible
% Every answer typed at a prompt must end with a period (e.g. 1.  Colombo.)
%
% PROGRAM MAP
%   1. Input helpers      safe reading of user input
%   2. Map data           ONE table road/4 (energy + km) - edit only here
%   3. A* heuristic       h/3 estimates + heuristic/3 wrapper
%   4. Search algorithms  dfs/4, bfs/4, astar/4
%   5. Tour planning      multi-target tour (menu option 6)
%   6. Drone state        battery, deliveries
%   7. Menu and options   1-10
%   8. Comparison + demo  option 8, option 9

%------------ Input helpers ------------
% safe_read/2 reads one term and never crashes on a syntax error.
% On bad syntax it returns '$invalid' instead of raising an exception.
safe_read(Term, Vars) :-
    catch(read_term(Term, [variable_names(Vars)]),
          error(syntax_error(_), _),
          ( Term = '$invalid', Vars = [] )).

% Prolog treats an unquoted capitalised word (e.g. Hambantota.) as a
% variable. variable_names/1 lets us recover the name the user typed.
resolve_var(Term, Vars, Name) :-
    var(Term),
    !,
    (   find_var_name(Term, Vars, Found)
    ->  Name = Found
    ;   Name = Term
    ).
resolve_var(Term, _, Term).

find_var_name(V, [N=V2|_], N) :- V == V2, !.
find_var_name(V, [_|T], N) :- find_var_name(V, T, N).

% Read one location name (quotes and capital letters are optional).
read_atom_input(Atom) :-
    safe_read(Term, Vars),
    resolve_var(Term, Vars, Name),
    canonical_location(Name, Atom).

% Match typed text with a real location name, ignoring upper/lower case.
canonical_location(In, Out) :-
    atom(In),
    downcase_atom(In, Low),
    known_location(Out),
    downcase_atom(Out, Low),
    !.
canonical_location(X, X).

known_location(L) :- ( edge(L, _, _) ; edge(_, L, _) ).

% Read the main-menu choice. An unbound variable (e.g. typing X.) must
% never match a menu clause, so it is turned into 'invalid'.
read_choice(Choice) :-
    safe_read(Term, Vars),
    resolve_var(Term, Vars, Choice0),
    (   var(Choice0) -> Choice = invalid ; Choice = Choice0 ).

% Read a yes/no style answer in lower case.
read_answer(Answer) :-
    safe_read(Term, Vars),
    resolve_var(Term, Vars, Name),
    (   atom(Name) -> downcase_atom(Name, Answer) ; Answer = invalid ).

%------------ Map data: ONE table for both energy and distance ------------
% road(CityA, CityB, EnergyCost, Km). Roads work in both directions.
% Energy: 500 units = a full battery (see full_charge_range/1).
% The energy values are estimates for terrain/wind, so they are not
% proportional to Km. If any Energy value is changed, run ?- check_heuristics.
road('Colombo',    'Kandy',       45, 95).
road('Kandy',      'Badulla',      27, 58).
road('Colombo',    'Badulla',    68, 134).
road('Colombo',    'Ratnapura',   32, 67).
road('Ratnapura',  'Badulla',      38, 79).
road('Ratnapura',  'Galle',        36, 74).
road('Ratnapura',  'Hambantota',  43, 91).
road('Galle',      'Hambantota',  50, 101).
road('Kandy',      'Plonnaruwa',  41, 83).
road('Colombo',    'Galle', 53, 106).
road('Hambantota', 'Galle',  50, 101).
road('Badulla', 'Hambantota',  48, 96).
road('Badulla', 'Polonnaruwa',  53, 106). %new added
road('Kandy',  'Ratnapura', 37, 72). %new added



edge(A, B, Energy) :- road(A, B, Energy, _).

%------------ A* heuristic values (estimated energy to the goal) ------------
% h(Node, Goal, Estimate). Every value must be <= the real cheapest cost,
% otherwise A* can return a non-optimal route.
h('Ratnapura', 'Hambantota', 43). 
h('Galle',     'Hambantota', 49).
h('Badulla',   'Hambantota', 47).

h('Ratnapura', 'Colombo',    31).
h('Kandy',     'Colombo',    44). 
h('Badulla',   'Colombo',    67).
h('Galle','Colombo',         52).

h('Kandy',     'Badulla',   26).
h('Colombo',   'Badulla',    67).
h('Ratnapura', 'Badulla',    37).
h('Polonnaruwa','Badulla', 52). %new added

h('Colombo',   'Ratnapura',  31).
h('Badulla',   'Ratnapura', 37).
h('Galle',     'Ratnapura',  35).
h('Hambantota','Ratnapura', 42).
h('Kandy','Ratnapura', 36). %new added

h('Ratnapura','Kandy', 36). %new added
h('Colombo','Kandy', 44).
h('Badulla','Kandy', 26).
h('Polonnaruwa','Kandy', 40).

h('Kandy','Polonnaruwa', 40).
h('Badulla','Polonnaruwa', 52). %new added

h('Colombo','Galle', 52).
h('Hambantota','Galle', 49).
h('Ratnapura','Galle', 35).






% Deterministic wrapper: exactly one estimate per node. Same node -> 0,
% unknown pair -> 0 (never overestimates, so A* stays optimal).
heuristic(A, B, H) :-
    (   A == B -> H = 0
    ;   h(A, B, H0) -> H = H0
    ;   H = 0
    ).

%------------ Road distances in kilometres ------------
road_km(A, B, Km) :- road(A, B, _, Km).

% Total distance of a path (roads work in either direction).
path_distance_km([_], 0).
path_distance_km([A, B | Rest], Km) :-
    (   road_km(A, B, Leg) ; road_km(B, A, Leg) ), !,
    path_distance_km([B | Rest], RestKm),
    Km is Leg + RestKm.

%------------ Battery and payload ------------
% Energy covered by a full battery.
full_charge_range(500).

% Maximum total package weight (kg) the drone can carry in one tour.
max_payload(100).

% Convert energy cost to a percentage of a full battery.
% ceiling/1 rounds UP so the battery use is never under-estimated.
battery_percent(Raw, Pct) :-
    full_charge_range(Range),
    Pct is ceiling(Raw * 100 / Range).

%------------ No-fly zones ------------
:- dynamic(blocked/2).
blocked('Ratnapura', 'Kandy'). % Bad weather.

%------------ Delivery points and package weights ------------

delivery_point('Hambantota', 15).
delivery_point('Badulla', 15).
delivery_point('Kandy', 20).
delivery_point('Polonnaruwa', 15).
delivery_point('Galle', 15).
delivery_point('Ratnapura', 20).




% Valid moves work in either direction unless blocked.

valid_move(Current, Next, Cost) :-
    (edge(Current, Next, Cost) ; edge(Next, Current, Cost)),
    \+ blocked(Current, Next),
    \+ blocked(Next, Current).

%------------ Search algorithms (return a route and energy cost) ------------

dfs(Start, Target, Path, Cost) :-
    dfs_helper(Start, Target, [Start], RevPath, Cost),
    reverse(RevPath, Path).

dfs_helper(Target, Target, Visited, Visited, 0).
dfs_helper(Current, Target, Visited, FinalPath, TotalCost) :-
    valid_move(Current, Next, StepCost),
    \+ member(Next, Visited),
    dfs_helper(Next, Target, [Next|Visited], FinalPath, RestCost),
    TotalCost is StepCost + RestCost.

% BFS minimizes the number of legs.
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

% A* uses estimated total energy cost.
astar(Start, Target, Path, Cost) :-
    heuristic(Start, Target, H),
    astar_search([[H,0,[Start]]], Target, RevPath, Cost),
    reverse(RevPath, Path).

astar_search([[_,Cost,[Target|Rest]]|_], Target, [Target|Rest], Cost).
astar_search([[_,G,[Current|Rest]]|Others], Target, Path, Cost) :-
    findall([F2, G2, [Next,Current|Rest]],
        (   valid_move(Current, Next, StepCost),
            \+ member(Next, [Current|Rest]),
            G2 is G + StepCost,
            heuristic(Next, Target, H),
            F2 is G2 + H),
        Children),
    append(Others, Children, All),
    sort(All, SortedQueue),
    astar_search(SortedQueue, Target, Path, Cost).

% Select the search algorithm (first solution only).
run_algo(dfs, S, T, P, C) :- once(dfs(S, T, P, C)).
run_algo(bfs, S, T, P, C) :- once(bfs(S, T, P, C)).
run_algo(astar, S, T, P, C) :- once(astar(S, T, P, C)).

%------------ Multi-target tour: visit each target in order, return to start ------------

plan_tour(Start, Targets, Algo, FullPath, TotalCost) :-
    append(Targets, [Start], FullRoute),
    plan_legs(Start, FullRoute, Algo, FullPath, TotalCost).

plan_legs(_, [], _, [], 0).
plan_legs(Current, [NextTarget|Rest], Algo, Path, TotalCost) :-
    run_algo(Algo, Current, NextTarget, LegPath, LegCost),
    plan_legs(NextTarget, Rest, Algo, RestPath, RestCost),
    TotalCost is LegCost + RestCost,
    combine_paths(LegPath, RestPath, Path).

% Combine paths without duplicating the waypoint.
combine_paths(P1, [], P1) :- !.
combine_paths(P1, [_|T2], Combined) :- append(P1, T2, Combined).

% Try every visiting order (A* legs) and keep the cheapest one.
% For more than 6 targets the number of orders is too large, so the
% entered order is used unchanged.
best_tour_order(Start, Targets, BestOrder, BestPath, BestCost) :-
    length(Targets, N),
    (   N =< 6
    ->  findall(Cost-Order-Path,
                (   permutation(Targets, Order),
                    plan_tour(Start, Order, astar, Path, Cost)
                ),
                Plans),
        Plans \== [],
        keysort(Plans, [BestCost-BestOrder-BestPath | _])
    ;   plan_tour(Start, Targets, astar, BestPath, BestCost),
        BestOrder = Targets
    ).

%------------ Current drone state ------------

max_battery(100).

:- dynamic(current_location/1).
:- dynamic(battery_level/1).
:- dynamic(delivered/1).   % Served locations.

% Reset position and battery.
reset_drone :-
    max_battery(Max),
    retractall(current_location(_)),
    retractall(battery_level(_)),
    assertz(current_location('Colombo')),
    assertz(battery_level(Max)).

% Start a new session (also forgets earlier deliveries).
init_drone :-
    retractall(delivered(_)),
    reset_drone.

available_location(Loc, Weight) :-
    delivery_point(Loc, Weight),
    \+ delivered(Loc).

%------------ Delivery menu ------------

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
    write('3. Block a route'), nl,
    write('4. Unblock a route'), nl,
    write('5. Show blocked routes'), nl,
    write('6. Plan a Multi-Target Tour'), nl,
    write('7. Reset drone (system reset)'), nl,
    write('8. Compare DFS / BFS / A* between two locations'), nl,
    write('9. Run automatic demo'), nl,
    write('10. Exit'), nl,
    write('Select an option (end with a period, e.g. 1.): '),
    read_choice(Choice),
    handle_choice(Choice).

handle_choice(1) :- !, view_delivery_details, menu_loop.
handle_choice(2) :- !, deliver_to_location, menu_loop.
handle_choice(3) :- !, block_road, menu_loop.
handle_choice(4) :- !, unblock_road, menu_loop.
handle_choice(5) :- !, show_blocked, menu_loop.
handle_choice(6) :- !, plan_tour_menu, menu_loop.
handle_choice(7) :- !, init_drone, nl,
    write('Drone reset: at Colombo, battery 100%, all deliveries cleared.'), nl, menu_loop.
handle_choice(8) :- !, compare_menu, menu_loop.
handle_choice(9) :- !, demo_steps, menu_loop.
handle_choice(10) :- !, nl, write('Exiting Drone Relief Delivery System. Safe travels!'), nl.
handle_choice(end_of_file) :- !, nl, write('Exiting Drone Relief Delivery System.'), nl.
handle_choice(_) :- nl, write('Invalid option, please try again.'), nl, menu_loop.

%------------ Option 6: Multi-target tour ------------

plan_tour_menu :-
    current_location(Start),
    battery_level(Bat),
    nl, write('--- Multi-Target Tour Planning ---'), nl,
    findall(L, available_location(L, _), Available),
    (   Available == []
    ->  write('All relief camps have been served. Nothing left to deliver.'), nl
    ;   format('Available locations: ~w~n', [Available]),
        max_payload(Payload),
        format('Drone payload limit: ~wkg~n', [Payload]),
        write('Enter targets separated by commas and end with a period (e.g., Colombo, Hambantota.): '),
        read_targets(RawTargets),
        (   validate_targets(RawTargets, Targets)
        ->  run_tour(Start, Targets, Bat)
        ;   true
        )
    ).

% Read comma-separated targets.
read_targets(Targets) :-
    safe_read(Term, Vars),
    resolve_tuple(Term, Vars, Resolved),
    tuple_to_list(Resolved, Targets0),
    maplist(canonical_location, Targets0, Targets).

% Resolve unquoted names anywhere inside a comma-separated tuple.
% var/1 must be checked before trying to match (A,B): unifying an
% unbound variable against that pattern would silently succeed by
% binding it into a fresh (_,_) structure, recursing forever.
resolve_tuple(Term, Vars, Resolved) :-
    (   var(Term)
    ->  resolve_var(Term, Vars, Resolved)
    ;   Term = (A, B)
    ->  resolve_tuple(A, Vars, RA),
        resolve_tuple(B, Vars, RB),
        Resolved = (RA, RB)
    ;   Resolved = Term
    ).

tuple_to_list((A, B), [A | Rest]) :-
    !, tuple_to_list(B, Rest).
tuple_to_list(A, List) :-
    (   ( A == end_of_file ; A == '$invalid' ; A == '' )
    ->  List = []
    ;   List = [A]
    ).

% Check the requested targets. Prints the reason and fails when invalid.
validate_targets([], _) :-
    !, nl, write('No valid targets specified.'), nl, fail.
validate_targets(Raw, Targets) :-
    list_to_set(Raw, Targets),
    (   member(T, Targets), \+ ( atom(T), available_location(T, _) )
    ->  nl,
        (   atom(T), delivered(T)
        ->  format('~w has already received its delivery.~n', [T])
        ;   format('~w is not a valid delivery location.~n', [T])
        ),
        fail
    ;   true
    ),
    tour_weight(Targets, Weight),
    max_payload(MaxPayload),
    (   Weight =< MaxPayload
    ->  true
    ;   nl, write('*** TOUR DENIED ***'), nl,
        format('Total package weight ~wkg exceeds the drone payload limit of ~wkg.~n',
               [Weight, MaxPayload]),
        fail
    ).

tour_weight(Targets, Total) :-
    findall(W, ( member(T, Targets), delivery_point(T, W) ), Ws),
    sum_list(Ws, Total).

% Compare the algorithms, then offer the cheapest tour for execution.
run_tour(Start, Targets, Bat) :-
    show_tour_comparison(Start, Targets, Bat),
    (   show_best_tour(Start, Targets, Bat, BestOrder, BestPath, Pct)
    ->  (   Pct =< Bat
        ->  nl,
            (   ask_confirmation('Fly this tour now? (yes. / no.): ')
            ->  execute_tour(Start, BestOrder, BestPath, Pct)
            ;   nl, write('Tour cancelled. No battery used.'), nl
            )
        ;   true
        )
    ;   write('No valid tour found (roads may be blocked).'), nl
    ).

% Table: DFS, BFS and A* on the targets in the order entered.
show_tour_comparison(Start, Targets, Bat) :-
    nl, write('--- Comparing Algorithms for the Tour (order as entered) ---'), nl,
    format('Visit order: ~w, then return to ~w~n', [Targets, Start]),
    report_tour(dfs, Start, Targets, Bat),
    report_tour(bfs, Start, Targets, Bat),
    report_tour(astar, Start, Targets, Bat).

% Cheapest visiting order (A* legs). Prints the plan and the battery check.
show_best_tour(Start, Targets, Bat, BestOrder, BestPath, Pct) :-
    nl, write('--- Best Visiting Order (A* legs, all orders tried) ---'), nl,
    best_tour_order(Start, Targets, BestOrder, BestPath, BestCost),
    battery_percent(BestCost, Pct),
    path_distance_km(BestPath, Km),
    tour_weight(BestOrder, Weight),
    format('Visit order: ~w~n', [BestOrder]),
    format('Full Route: ~w~n', [BestPath]),
    format('Distance: ~w km | Battery Cost: ~w% | Total Weight: ~wkg~n', [Km, Pct, Weight]),
    format('Battery Available: ~w%~n', [Bat]),
    (   Pct =< Bat
    ->  Remaining is Bat - Pct,
        format('Battery After Tour: ~w%~n', [Remaining])
    ;   nl, write('*** TOUR DENIED ***'), nl,
        format('Insufficient battery for this tour. Required: ~w%, Available: ~w%~n', [Pct, Bat]),
        write('Try fewer targets, or reset the drone (option 7).'), nl
    ).

% One line of the comparison table.
report_tour(Algo, Start, Targets, Bat) :-
    (   plan_tour(Start, Targets, Algo, Path, Cost)
    ->  battery_percent(Cost, Pct),
        path_distance_km(Path, Km),
        (   Pct =< Bat -> Status = 'OK' ; Status = 'NOT ENOUGH BATTERY' ),
        format('~w~t~7| Distance: ~w km | Battery Cost: ~w% (~w)~n       Full Route: ~w~n',
               [Algo, Km, Pct, Status, Path])
    ;   format('~w~t~7| No valid tour found.~n', [Algo])
    ).

% Fly the tour: use battery and mark every target as served.
% The drone returns to the base, so current_location does not change.
execute_tour(Start, Order, Path, Pct) :-
    battery_level(Bat),
    NewBat is Bat - Pct,
    retract(battery_level(Bat)),
    assertz(battery_level(NewBat)),
    forall(member(T, Order), assertz(delivered(T))),
    tour_weight(Order, Weight),
    nl, write('*** TOUR APPROVED ***'), nl,
    format('Route Taken: ~w~n', [Path]),
    format('Delivered ~wkg of supplies to ~w and returned to ~w.~n', [Weight, Order, Start]),
    format('Battery Used: ~w%   |   Battery Remaining: ~w%~n', [Pct, NewBat]).

%------------ Option 8: compare the three algorithms ------------

compare_menu :-
    nl, write('--- Algorithm Comparison ---'), nl,
    write('Enter start location (end with a period, e.g. Ratnapura.): '),
    read_atom_input(Start),
    write('Enter goal location (end with a period, e.g. Hambantota.): '),
    read_atom_input(Goal),
    (   ( var(Start) ; var(Goal) ; \+ known_location(Start) ; \+ known_location(Goal) )
    ->  nl, write('Unknown location. Known locations: '), write_known_locations, nl
    ;   compare_algorithms(Start, Goal)
    ).

write_known_locations :-
    findall(L, known_location(L), All),
    sort(All, Sorted),
    write(Sorted).

% Shows every DFS path, then one line per algorithm (path, cost, work).
compare_algorithms(Start, Goal) :-
    nl, format('--- DFS, BFS and A* : ~w to ~w ---~n', [Start, Goal]),
    findall(C-P, dfs(Start, Goal, P, C), AllPaths),
    (   AllPaths == []
    ->  write('No path found (roads may be blocked).'), nl
    ;   length(AllPaths, N),
        format('All possible loop-free paths (~w found by DFS):~n', [N]),
        forall(member(C-P, AllPaths),
               format('  cost ~w : ~w~n', [C, P])),
        nl,
        findall(Algo-Cost-Legs,
                ( member(Algo, [dfs, bfs, astar]),
                  measure_algo(Algo, Start, Goal, Path, Cost, Work),
                  length(Path, Len), Legs is Len - 1,
                  path_distance_km(Path, Km),
                  battery_percent(Cost, Pct),
                  format('~w~t~6| legs: ~w | energy: ~w | distance: ~w km | battery: ~w% | work: ~w inferences~n',
                         [Algo, Legs, Cost, Km, Pct, Work]),
                  format('       route: ~w~n', [Path])
                ),
                Results),
        summarise_comparison(Results)
    ).

measure_algo(Algo, Start, Goal, Path, Cost, Work) :-
    statistics(inferences, I0),
    run_algo(Algo, Start, Goal, Path, Cost),
    statistics(inferences, I1),
    Work is I1 - I0.

summarise_comparison(Results) :-
    findall(C, member(_-C-_, Results), Costs), min_list(Costs, MinC),
    findall(L, member(_-_-L, Results), Legs),  min_list(Legs, MinL),
    findall(A, member(A-MinC-_, Results), CheapAlgos),
    findall(A, member(A-_-MinL, Results), ShortAlgos),
    nl,
    format('Lowest energy cost (~w) : ~w~n', [MinC, CheapAlgos]),
    format('Fewest legs (~w)        : ~w~n', [MinL, ShortAlgos]).

%------------ Option 9: automatic demonstration ------------

demo :- init_drone, demo_steps.

demo_steps :-
    nl, write('#### DEMO 1: normal conditions - compare DFS, BFS and A* ####'), nl,
    compare_algorithms('Ratnapura', 'Hambantota'),
    nl, write('#### DEMO 2: road Balangoda-Badulla is blocked - routes change ####'), nl,
    (   blocked('Balangoda', 'Badulla') -> Added = false
    ;   assertz(blocked('Balangoda', 'Badulla')), Added = true
    ),
    compare_algorithms('Ratnapura', 'Hambantota'),
    (   Added == true -> retractall(blocked('Balangoda', 'Badulla')) ; true ),
    write('(Road unblocked again after the demo.)'), nl,
    nl, write('#### DEMO 3: multi-target tour Colombo, Hambantota, Badulla ####'), nl,
    max_battery(Full),
    show_tour_comparison('Ratnapura', ['Colombo', 'Hambantota', 'Badulla'], Full),
    show_best_tour('Ratnapura', ['Colombo', 'Hambantota', 'Badulla'], Full, _, _, _),
    nl, write('(Demo only - no battery used, nothing marked as delivered.)'), nl.

%------------ Heuristic check (admissibility) ------------
% An A* heuristic must never be larger than the real cheapest cost.
% This ignores blocked roads (blocking can only make real costs bigger).

check_heuristics :-
    nl, write('--- Checking A* heuristic values ---'), nl,
    findall(N-G-V-Real,
            ( h(N, G, V),
              aggregate_all(min(C), raw_path_cost(N, G, [N], C), Real),
              V > Real ),
            Bad),
    aggregate_all(count, h(_, _, _), Total),
    (   Bad == []
    ->  format('All ~w heuristic values are admissible (never overestimate).~n', [Total])
    ;   forall(member(N-G-V-Real, Bad),
               format('TOO HIGH: h(~w, ~w) = ~w but real cost is ~w~n', [N, G, V, Real]))
    ).

raw_path_cost(G, G, _, 0).
raw_path_cost(N, G, Seen, Cost) :-
    (   edge(N, X, Step) ; edge(X, N, Step) ),
    \+ memberchk(X, Seen),
    raw_path_cost(X, G, [X|Seen], Rest),
    Cost is Step + Rest.

%------------ Block or unblock a road ------------

block_road :-
    nl, write('Enter Road to Block (example: Ratnapura. Kandy.): '),
    read_atom_input(A), read_atom_input(B),
    (   (var(A) ; var(B))
    ->  nl, write('Invalid input: could not read those location names.'), nl
    ;   known_road(A, B)
    ->  (   ( blocked(A, B) ; blocked(B, A) )
        ->  nl, write('That road is already blocked.'), nl
        ;   assertz(blocked(A, B)),
            nl, write('Road blocked: '), write(A-B), nl
        )
    ;   nl, write('Unknown road. No road was blocked.'), nl
    ).

known_road(A, B) :- edge(A, B, _).
known_road(A, B) :- edge(B, A, _).

unblock_road :-
    nl, write('Enter Road to Unblock (example: Ratnapura. Kandy.): '),
    read_atom_input(A), read_atom_input(B),
    (   (var(A) ; var(B))
    ->  nl, write('Invalid input: could not read those location names.'), nl
    ;   known_road(A, B),
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

%------------ Options 1 and 2: single deliveries ------------

% Shared summary block: round-trip distance/cost, outbound route, weight.
show_location_summary(Current, Loc, Weight) :-
    (   astar(Current, Loc, OutPath, OutCost),
        astar(Loc, Current, ReturnPath, ReturnCost)
    ->  TotalCost is OutCost + ReturnCost,
        path_distance_km(OutPath, OutKm),
        path_distance_km(ReturnPath, ReturnKm),
        TotalKm is OutKm + ReturnKm,
        battery_percent(TotalCost, Pct),
        format('Location: ~w~n', [Loc]),
        format('  Distance: ~w km (round trip)~n', [TotalKm]),
        format('  Route: ~w~n', [OutPath]),
        format('  Battery Cost: ~w% (round trip)~n', [Pct]),
        format('  Package Weight: ~wkg~n', [Weight])
    ;   format('Location: ~w | No valid round-trip path from ~w.~n', [Loc, Current])
    ).

view_delivery_details :-
    current_location(Current),
    nl, write('--- Delivery Locations (goods to be delivered) ---'), nl,
    format('(Calculated from current location: ~w, round trip)~n', [Current]),
    (   \+ available_location(_, _)
    ->  write('All relief camps have been served. No pending deliveries.'), nl
    ;   forall(
            available_location(Loc, Weight),
            show_location_summary(Current, Loc, Weight)
        )
    ).

deliver_to_location :-
    findall(L, available_location(L, _), Locations),
    (   Locations == []
    ->  nl, write('All relief camps have been served. Nothing left to deliver.'), nl
    ;   current_location(Current),
        nl, write('Available locations: '), write(Locations), nl,
        write('Enter destination location (end with a period, e.g. Colombo.): '),
        read_atom_input(Target),
        (   var(Target)
        ->  nl, write('Invalid input: could not read that location name.'), nl
        ;   available_location(Target, TargetWeight)
        ->  (   astar(Current, Target, OutPath, OutCost),
                astar(Target, Current, ReturnPath, ReturnCost)
            ->  review_and_confirm(Current, Target, OutPath, OutCost, ReturnPath, ReturnCost, TargetWeight)
            ;   format('No valid round-trip path between ~w and ~w. Delivery not possible.~n', [Current, Target])
            )
        ;   delivered(Target)
        ->  nl, format('~w has already received its delivery.~n', [Target])
        ;   nl, write('Invalid delivery location.'), nl
        )
    ).

review_and_confirm(Current, Target, OutPath, OutCost, ReturnPath, ReturnCost, Weight) :-
    TotalCost is OutCost + ReturnCost,
    battery_percent(TotalCost, Pct),
    battery_level(Bat),
    combine_paths(OutPath, ReturnPath, FullPath),
    path_distance_km(OutPath, OutKm),
    path_distance_km(ReturnPath, ReturnKm),
    TotalKm is OutKm + ReturnKm,
    nl, write('--- Delivery Details (round trip) ---'), nl,
    format('Location: ~w~n', [Target]),
    format('  Distance: ~w km (round trip)~n', [TotalKm]),
    format('  Route: ~w~n', [OutPath]),
    format('  Battery Cost: ~w% (round trip)~n', [Pct]),
    format('  Package Weight: ~wkg~n', [Weight]),
    format('  Battery Available: ~w%~n', [Bat]),
    (   Pct =< Bat
    ->  Remaining is Bat - Pct,
        format('  Battery After Delivery: ~w%~n', [Remaining]),
        nl,
        (   ask_confirmation('Confirm this delivery? (yes. / no.): ')
        ->  execute_delivery(Current, Target, FullPath, Pct, Weight)
        ;   nl, write('Delivery cancelled. No battery used.'), nl
        )
    ;   nl, write('*** DELIVERY DENIED ***'), nl,
        format('Insufficient battery for this round trip. Required: ~w%, Available: ~w%~n', [Pct, Bat])
    ).

% Ask a yes/no question. Capital letters (Yes. / No.) are accepted, and an
% unbound variable can no longer count as "yes".
ask_confirmation(Prompt) :-
    write(Prompt),
    read_answer(Answer),
    (   memberchk(Answer, [yes, y])
    ->  true
    ;   memberchk(Answer, [no, n, end_of_file])
    ->  fail
    ;   write('Please answer yes. or no.'), nl,
        ask_confirmation(Prompt)
    ).

execute_delivery(Current, Target, Path, Pct, Weight) :-
    battery_level(Bat),
    NewBat is Bat - Pct,
    retract(battery_level(Bat)),
    assertz(battery_level(NewBat)),
    % The drone flies out, delivers, and flies back; current_location(Current)
    % stays as-is since the round trip ends where it started.
    assertz(delivered(Target)),   % Mark the location as served.
    nl, write('*** DELIVERY APPROVED ***'), nl,
    format('Route Taken (round trip): ~w~n', [Path]),
    format('Battery Used: ~w%   |   Battery Remaining: ~w%~n', [Pct, NewBat]),
    format('Delivered ~wkg of supplies to ~w and returned to ~w.~n', [Weight, Target, Current]).
