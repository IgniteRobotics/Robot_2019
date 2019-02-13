package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

public abstract class IgniteSubsystem extends Subsystem {

    public abstract void writeToLog();

    public abstract boolean checkSystem();

    public abstract void outputTelemetry();

    public void zeroSensors() {
    }
    
    public void establishDefaultCommand() {
    }

}