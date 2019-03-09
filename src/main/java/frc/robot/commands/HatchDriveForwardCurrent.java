/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.DriveTrain;

public class HatchDriveForwardCurrent extends Command {

  private Carriage carriage;
  private DriveTrain driveTrain;

  private final double CURRENT_THRESHOLD = 15;

  public HatchDriveForwardCurrent(Carriage carriage, DriveTrain driveTrain) {

    this.carriage = carriage;
    this.driveTrain = driveTrain;

    requires(this.carriage);
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    //driveTrain.arcadeDrive(0.5, 0, 0);

    carriage.openBeak();

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    return driveTrain.getLeftMasterCurrent() > CURRENT_THRESHOLD;

  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    carriage.retractBeak();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
  
}
