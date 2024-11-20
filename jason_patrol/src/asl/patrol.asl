/* Initial beliefs and rules */
init(r2d2).
at(r2d2).
visit(r2d2, wp1).
visit(r2d2, wp2).
visit(r2d2, wp3).
visit(r2d2, wp4).

/* Initial goals */

// !patrol.

/* Plans */

+!patrol            : at(Loc) & visit(Loc, WP) & not moving(_) <- .print("Moving to waypoint ", WP); +moving(WP); -movebase_result(3)[source(percept)]; move(WP).
+!patrol            : init(Base) & not at(Base) <- .print("Moving back to initial position."); +moving(Base); -movebase_result(3)[source(percept)]; move(Base);.
// if we got here it means we have visited all waypoints, time to add them back
// +!patrol            : init(Base) <- +visit(Base,wp1); +visit(Base,wp2); +visit(Base,wp3); +visit(Base,wp4); !patrol.

+start: true <- !patrol.

+movebase_result(3) : moving(WP) & visit(r2d2, _) <- .print("Movement completed, resuming patrol."); -moving(WP); -visit(_, WP); -at(_); +at(WP); !patrol.
+movebase_result(3) : moving(WP) & not visit(r2d2, _) <- .print("Movement completed, no more patrol."); -moving(WP); -visit(_, WP); -at(_); +at(WP); end.
+movebase_result(2) : moving(WP) <- .print("Movement ended with failure."); -moving(WP).
