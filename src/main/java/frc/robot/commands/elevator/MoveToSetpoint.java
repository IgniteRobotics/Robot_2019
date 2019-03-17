/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.CarriageLevel;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;

public class MoveToSetpoint extends Command {

  private Elevator elevator;
  private Carriage carriage;
  private CarriageLevel level;

  public MoveToSetpoint(Elevator elevator, CarriageLevel level, Carriage carriage) {

    this.elevator = elevator;
    this.level = level;
    this.carriage = carriage;
    
    requires(this.elevator);
    
  }


  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    int setPoint = carriage.getSetpoint(level);
    elevator.setMotionMagicPosition(setPoint);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return elevator.isMotionMagicDone();
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
