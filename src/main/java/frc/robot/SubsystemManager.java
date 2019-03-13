package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.robot.subsystems.IgniteSubsystem;

public class SubsystemManager{

    private List<IgniteSubsystem> subsystems;

    public SubsystemManager() {
        subsystems = new ArrayList<>();
    }

	public void addSubsystems(IgniteSubsystem... subsystems) {
		this.subsystems.addAll(Arrays.asList(subsystems));
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