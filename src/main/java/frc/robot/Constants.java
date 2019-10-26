package frc.robot;

public class Constants {

    // intake
    public static final double OUTTAKE_POWER = 0.5;
    public static final double INTAKE_POWER = -0.5;

    // drivetrain
    public static final double DRIVE_DEADBAND = 0.03; 
    public static final double EXP_BASE = 5.0;
    public static final double EJECT_THEN_HOME_DRIVE_TIME = 0.5;
    public static final double EJECT_THEN_HOME_DRIVE_POWER = -0.2;

    // elevator
    public static final double ELEVATOR_JOG_DEADBAND = 0.1;

    public static final double EJECT_TIMEOUT = 0.5;
    public static final double CARGOSHIP_CARGO_EJECT_TIMEOUT = 0.5;

    public static final int ROCKET_HATCH_L3 = -38000;
    public static final int ROCKET_HATCH_L2 = -20000;
    public static final int ROCKET_HATCH_L1 = -4000;

    public static final int ROCKET_CARGO_L3 = -42500;
    public static final int ROCKET_CARGO_L2 = -25500;
    public static final int ROCKET_CARGO_L1 = -9175;

    public static final int CARGO_SHIP_CARGO = -14500 - 4000 - 1000;
    public static final int CARGO_SHIP_HATCH = 0;
    public static final int HATCH_PICKUP = -5000;   // was -6000

}