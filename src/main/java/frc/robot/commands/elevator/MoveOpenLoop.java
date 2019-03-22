/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;

public class MoveOpenLoop extends Command {

  private Elevator elevator;

  private Joystick manipulatorJoystick;
  private final int THROTTLE_AXIS;
  private final double DEADBAND;

  public MoveOpenLoop(Elevator elevator, Joystick manipulatorJoystick, int throttleId, double deadband) {

    this.elevator = elevator;

    this.THROTTLE_AXIS = throttleId;
    this.DEADBAND = deadband;

    this.manipulatorJoystick = manipulatorJoystick;

    requires(this.elevator);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    elevator.setState(ElevatorState.Unknown);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double throttle = manipulatorJoystick.getRawAxis(THROTTLE_AXIS);
    elevator.setOpenLoop(throttle, DEADBAND);
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    elevator.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
