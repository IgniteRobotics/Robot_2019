/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;


/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  
  private WPI_VictorSPX intakeMotor;
  private Solenoid intake;

  private Command defaultCommand;

  public Intake(int pcmID, int intakeMotorID, int intakeSolenoid) {

    intakeMotor = new WPI_VictorSPX(intakeMotorID);
    intake = new Solenoid(pcmID, intakeSolenoid);

  }

  public void setCommandDefault(Command command){
    this.defaultCommand = command;
    initDefaultCommand();
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(this.defaultCommand);
  }
}
