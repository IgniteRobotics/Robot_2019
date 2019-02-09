/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.driveTrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class ArcadeDrive extends Command {

  private Drivetrain driveTrain;
  private XboxController xboxController;

  public ArcadeDrive(Drivetrain driveTrain, XboxController xboxController) {

    this.driveTrain = driveTrain;
    this.xboxController = xboxController;
    
    requires(this.driveTrain);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double throttle = xboxController.getY(Hand.kLeft);
    double rotation = xboxController.getX(Hand.kRight);

    driveTrain.arcadeDrive(throttle, rotation);
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
