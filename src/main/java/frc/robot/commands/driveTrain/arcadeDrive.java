/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.driveTrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.DriveTrain;

public class arcadeDrive extends Command {

  private DriveTrain driveTrain;

	private final int THROTTLE_AXIS;
	private final int TURN_AXIS;
  private final double DEADBAND;

  private Joystick driverJoystick;

  public arcadeDrive(DriveTrain driveTrain, Joystick driverJoystick, int throttleId, int turnId, double deadband) {

    this.driveTrain = driveTrain;

		this.THROTTLE_AXIS = throttleId;
		this.TURN_AXIS = turnId;
    this.DEADBAND = deadband;

    this.driverJoystick = driverJoystick;

    requires(this.driveTrain);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    return;
    // double throttle = driverJoystick.getRawAxis(THROTTLE_AXIS);
    // double rotation = driverJoystick.getRawAxis(TURN_AXIS);

    // driveTrain.arcadeDrive(-throttle, rotation, DEADBAND);
  
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
