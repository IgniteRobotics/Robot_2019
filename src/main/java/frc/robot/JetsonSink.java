package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JetsonSink {

    private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static NetworkTable table = inst.getTable("Vision");

    public double getTurn1() {
        return (double)table.getEntry("turn1_angle").getNumber(0);
    }

    public double getDistance1() {
        return (double)table.getEntry("calc_c_side").getNumber(0);
    }

    public double getTurn2() {
        return (double)table.getEntry("turn2_angle").getNumber(0);
    }
    
    public double getDistance2() {
        return (double)table.getEntry("TARGET_AIM_OFFSET").getNumber(0);
    }

    public void outputTelemetry() {
        SmartDashboard.putNumber("turn1", this.getTurn1());
        SmartDashboard.putNumber("distance1", this.getDistance1());
        SmartDashboard.putNumber("turn2", this.getTurn2());
        SmartDashboard.putNumber("distance2", this.getDistance2());
    }


}