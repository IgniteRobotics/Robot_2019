/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Intake;

public class OpenIntakeUntilBeam extends Command {

  private Intake intake;
  private Carriage carriage;
  private double power;

  public OpenIntakeUntilBeam(Intake intake, Carriage carriage, double power) {

    this.intake = intake;
    this.carriage = carriage;

    this.power = power;

    requires(this.intake);
    requires(this.carriage);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    intake.setOpenLoop(power);
    intake.openIntake();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return intake.hasCargo();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    intake.closeIntake();
    intake.stopIntakeMotor();
  }

}
