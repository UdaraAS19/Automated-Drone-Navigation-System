%    ---Automated Drone Navigation System---


%     ---flight paths---

path('Ratnapura', 'Kalutara', 55).
path('Ratnapura', 'Balangoda', 42).
path('Ratnapura', 'Avissawella', 45).
path('Kalutara', 'Galle', 70).
path('Kalutara', 'Colombo', 43).
path('Avissawella', 'Colombo', 36).
path('Avissawella', 'Kandy', 65).
path('Balangoda', 'Badulla', 68).
path('Galle', 'Matara', 35).
path('Matara', 'Hambantota', 75).
path('Badulla', 'Hambantota', 110).
path('Kandy', 'Badulla', 105).

%     Flight path can be traveled in both directions

flight_route(X, Y, D) :- path(X, Y, D).
flight_route(X, Y, D) :- path(Y, X, D).
