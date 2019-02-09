/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Carriage extends Subsystem {

  private Solenoid cargoEject;
  private Solenoid beak;
  private Solenoid hatchEject; 
  private DigitalInput beamBreak;

  private Command defaultCommand;

  public Carriage(int pcmID, int cargoEjectSolenoid, int beakSolenoid, int hatchEjectSolenoid, int beamBreakID) {

    cargoEject = new Solenoid(pcmID, cargoEjectSolenoid);
    beak = new Solenoid(pcmID, beakSolenoid);
    hatchEject = new Solenoid(pcmID, hatchEjectSolenoid);
    beamBreak = new DigitalInput(beamBreakID);

  }

	public void setCommandDefault(Command command) {
		this.defaultCommand = command;
		initDefaultCommand();
	}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

}
