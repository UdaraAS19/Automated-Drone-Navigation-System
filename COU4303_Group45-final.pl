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
