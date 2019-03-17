package frc.robot;

public class Constants {

    //drivetrain
    public static final double DRIVE_DEADBAND = 0.03;

    //elevator
    public static final double ELEVATOR_JOG_DEADBAND = 0.1;

    public static final double EJECT_TIMEOUT = 0.5;
    public static final double CARGOSHIP_CARGO_EJECT_TIMEOUT = 0.6829;

    public static final int ROCKET_HATCH_L3 = -38000;
    public static final int ROCKET_HATCH_L2 = -20000;
    public static final int ROCKET_HATCH_L1 = -4000;

    public static final int ROCKET_CARGO_L3 = -42500;
    public static final int ROCKET_CARGO_L2 = -25500;
    public static final int ROCKET_CARGO_L1 = -9175;

    public static final int CARGO_SHIP_CARGO = -14500 - 4000 - 1000;
    public static final int CARGO_SHIP_HATCH = ROCKET_HATCH_L1;
    public static final int HATCH_PICKUP = -6000;

}