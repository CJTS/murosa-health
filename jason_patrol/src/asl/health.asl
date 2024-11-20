/* Initial beliefs and rules */
+robot_at(r, room2).
+nurse_at(n, room1).
+arm_at(a, room3).
+nurse_has_sample(n).
+opened_door(room1).
+opened_door(room2).
+opened_door(room3).

/* Initial goals */
+!goal <-
    arm_has_sample(a).

/* Plans */
// Navegar entre salas
+!navigate(From, To) : robot_at(From) & opened_door(From) & opened_door(To) <- 
    .print("Navegando de", From, "para", To);
    -robot_at(From);
    +robot_at(To).

// Abrir porta
+!open_door(Room, Nurse) : nurse_at(Nurse, Room) & not opened_door(Room) <- 
    .print("Abrindo a porta da sala", Room);
    +opened_door(Room).

// Aproximar-se da enfermeira
+!approach_nurse(Robot, Nurse, Room) : robot_at(Room) & nurse_at(Nurse, Room) & not robot_near_nurse(Robot, Nurse) <- 
    .print("Aproximando-se da enfermeira", Nurse, "na sala", Room);
    +robot_near_nurse(Robot, Nurse).

// Autenticar enfermeira
+!authenticate_nurse(Robot, Nurse) : robot_near_nurse(Robot, Nurse) & not nurse_auth_robot(Robot, Nurse) <- 
    .print("Autenticando com a enfermeira", Nurse);
    +nurse_auth_robot(Robot, Nurse).

// Depositar amostra do enfermeiro para o robô
+!deposit_sample(Nurse, Robot, Room) : robot_drawer(Robot) & nurse_auth_robot(Robot, Nurse) & nurse_has_sample(Nurse) <- 
    .print("Transferindo amostra da enfermeira para o robô.");
    -nurse_has_sample(Nurse);
    +robot_has_sample(Robot).

// Aproximar-se do braço mecânico
+!approach_arm(Robot, Arm, Room) : robot_at(Room) & arm_at(Arm, Room) & not robot_near_arm(Robot, Arm) <- 
    .print("Aproximando-se do braço mecânico", Arm, "na sala", Room);
    +robot_near_arm(Robot, Arm).

// Pegar amostra do robô para o braço mecânico
+!pick_up_sample(Arm, Robot, Room) : robot_has_sample(Robot) & robot_near_arm(Robot, Arm) <- 
    .print("Transferindo amostra do robô para o braço mecânico.");
    -robot_has_sample(Robot);
    +arm_has_sample(Arm).

+start: true <- !arm_has_sample.