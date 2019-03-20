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
    }

    public void turnOffLed() {
        led.set(Relay.Value.kOff);
    }

    public void turnOnLed() {
        led.set(Relay.Value.kOn);
    }

    public double getTurn1() {
        return (double)table.getEntry("TURN_1").getNumber(0);
    }

    public double getDistance1() {
        return (double)table.getEntry("DISTANCE_1").getNumber(0);
    }

    public double getTurn2() {
        return (double)table.getEntry("TURN_2").getNumber(0);
    }
    
    public double getDistance2() {
        return (double)table.getEntry("DISTANCE_2").getNumber(0);
    }

    public double getDirectTurn() {
        return (double)table.getEntry("DIRECT_TURN").getNumber(0);
    }

    public double getDirectDistance() {
        return (double)table.getEntry("DIRECT_DISTANCE").getNumber(0);
    }

    public void turnOn() {
    for (int i = 0; i < 10 ; i++) {
        jetsonPower.set(true);
      }
      for (int i = 0; i < 10 ; i++) {
        jetsonPower.set(false);
      }
      jetsonPower.set(true);

      SmartDashboard.putBoolean("Jetson Power", true);
    }

    public void turnOffToggle() {
        jetsonPowerState = SmartDashboard.getBoolean("Jetson Power", true);
        jetsonPower.set(jetsonPowerState);
    }
    
    public void outputTelemetry() {
        SmartDashboard.putNumber("turn 1", this.getTurn1());
        SmartDashboard.putNumber("distance 1", this.getDistance1());
        SmartDashboard.putNumber("turn 2", this.getTurn2());
        SmartDashboard.putNumber("distance 2", this.getDistance2());
        SmartDashboard.putNumber("direct distance", this.getDirectDistance());
        SmartDashboard.putNumber("direct turn", this.getDirectTurn());
    }

    public void zeroSensors() {}
    
    public boolean checkSystem() {
        return true;
    }

    public void writeToLog() {}
    
    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(null);  
    }

}