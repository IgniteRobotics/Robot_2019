package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IgniteSubsystem;

public class SubsystemManager {

	private List<IgniteSubsystem> subsystems;
	private boolean zeroSensors;

	public SubsystemManager() {
		subsystems = new ArrayList<>();
		SmartDashboard.putBoolean("Zero sensors", false);
	}

	public void addSubsystems(IgniteSubsystem... subsystems) {
		this.subsystems.addAll(Arrays.asList(subsystems));
	}

	public void zeroSensorsFromDashboard() {
		zeroSensors = SmartDashboard.getBoolean("Zero sensors", false);
		if (zeroSensors) {
			zeroAllSensors();
		}
	}

	public void zeroAllSensors() {
		for (IgniteSubsystem s : subsystems) {
			s.zeroSensors();
		}
	}

	public void outputTelemetry() {
		for (IgniteSubsystem s : subsystems) {
			s.outputTelemetry();
		}
	}

}