package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Jetson extends IgniteSubsystem {

    private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static NetworkTable table = inst.getTable("Vision");

    private DigitalOutput jetsonPower;

    private Relay led;

    private boolean jetsonPowerState;

    public Jetson(int jetsonPowerDioID, int relayID) {
        jetsonPower = new DigitalOutput(jetsonPowerDioID);
        led = new Relay(relayID);
        SmartDashboard.putBoolean("Jetson Power", true);
    }

    public void turnOffLed() {
        led.set(Relay.Value.kOff);
    }

    public void turnOnLed() {
        led.set(Relay.Value.kOn); // TODO: FOR COMP BOT
        // led.set(Relay.Value.kForward); // TODO: FOR PRACTICE BOT
    }

//     public String gettimetable() {
//         return (String)table.getEntry("timetable").getString("yeet");
//     }

    public double getTurn1() {
        return (double) table.getEntry("TURN_1").getNumber(0);
    }

    public double getDistance1() {
        return (double) table.getEntry("DISTANCE_1").getNumber(0);
    }

    public double getTurn2() {
        return (double) table.getEntry("TURN_2").getNumber(0);
    }

    public double getDistance2() {
        return (double) table.getEntry("DISTANCE_2").getNumber(0);
    }

    // public double getDirectTurn() {
    //     return (double) table.getEntry("DIRECT_TURN").getNumber(0);
    // }

    // public double getDirectDistance() {

    //     return (double) table.getEntry("DIRECT_DISTANCE").getNumber(0);
    // }

    public double getTargetAngle() {
        return (double) table.getEntry("X_ANGLE").getNumber(0);
    }
    

    public void turnOn() {
        for (int i = 0; i < 10; i++) {
            jetsonPower.set(true);
        }
        for (int i = 0; i < 10; i++) {
            jetsonPower.set(false);
        }
        jetsonPower.set(true);

        SmartDashboard.putBoolean("Jetson Power", true);
    }

    public void turnOffToggle() {
        jetsonPowerState = SmartDashboard.getBoolean("Jetson Power", true);
        jetsonPower.set(jetsonPowerState);
    }

    public void outputTelemetry() {}

    public void zeroSensors() {
    }

    public boolean checkSystem() {
        return true;
    }

    public void writeToLog() {
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(null);
    }

}