+start(Patrol, Base, Room1): true <- +start(Patrol, Base, Room1); !move(Patrol, Room1).

+!move(Patrol, Room1): start(Patrol, Base, Room1) <- move(Patrol, Room1).
+success_move(Patrol, Room1): start(Patrol, Base, Room1) <- +milestone0; !move(Patrol, Base).
+!move(Patrol, Base): milestone0 <- move(Patrol, Base).
+success_move(Patrol, Base): start(Patrol, Base, Room1) & milestone0 <- -milestone0; end.
