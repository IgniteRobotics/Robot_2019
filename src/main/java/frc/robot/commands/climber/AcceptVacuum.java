/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Climber;

public class AcceptVacuum extends Command {


  private Climber climber;
  private Joystick manipulatorJoystick;
  private final int THROTTLE_AXIS;
  private final double DEADBAND;
  private int acceptButtonId;

  public AcceptVacuum(Climber climber, Joystick manipulatorJoystick, int throttleId, int acceptButtonId, double deadband) {

    this.THROTTLE_AXIS = throttleId;
    this.DEADBAND = deadband;
    this.acceptButtonId = acceptButtonId;
    this.manipulatorJoystick = manipulatorJoystick;
    this.climber = climber;
    requires(this.climber);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double throttle = manipulatorJoystick.getRawAxis(THROTTLE_AXIS);
    climber.setOpenLoop(throttle, DEADBAND);
  }

  // End command when the accept button is pushed.
  @Override
  protected boolean isFinished() {
    boolean accepted = manipulatorJoystick.getRawButton(acceptButtonId);
    boolean atLimit = (climber.isRevLimitTripped() || climber.isFwdLimitTripped());
    return accepted || atLimit;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if (climber.isFwdLimitTripped()) {
      climber.zeroSensors();
    }
    climber.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }


  
}
