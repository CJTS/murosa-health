+robot(Name) : true <-
    .print("Robot ", Name, " created");
    .create_agent(Name, "robot.asl").

+arm(Name) : true <-
    .print("Arm ", Name, " created");
    .create_agent(Name, "arm.asl").

+nurse(Name) : true <-
    .print("Nurse ", Name, " created");
    .create_agent(Name, "nurse.asl").