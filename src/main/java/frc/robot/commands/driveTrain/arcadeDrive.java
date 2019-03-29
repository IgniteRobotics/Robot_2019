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
import frc.robot.subsystems.Jetson;

public class arcadeDrive extends Command {

  private DriveTrain driveTrain;

  private final int THROTTLE_AXIS;
  private final int TURN_AXIS;
  private final double DEADBAND;

  private final double kP = 2.0;

  private Joystick driverJoystick;

  private Jetson jetson;

  public arcadeDrive(DriveTrain driveTrain, Joystick driverJoystick, int throttleId, int turnId, double deadband, Jetson jetson) {

    this.driveTrain = driveTrain;

    this.THROTTLE_AXIS = throttleId;
    this.TURN_AXIS = turnId;
    this.DEADBAND = deadband;

    this.driverJoystick = driverJoystick;

    this.jetson = jetson;

    requires(this.driveTrain);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double throttle = driverJoystick.getRawAxis(THROTTLE_AXIS);
    double rotation = driverJoystick.getRawAxis(TURN_AXIS);

    
    if (driverJoystick.getRawButton(2)) { //button pushed
      double targetAngle = 0;
      targetAngle = jetson.getDirectTurn();
      if (targetAngle != 0) { // got a value in NT
        double angle = limitOutput((kP * targetAngle), 0.45);  
        driveTrain.arcadeDrive(-throttle*.6, angle, DEADBAND);
      } else {
        driveTrain.arcadeDrive(-throttle, rotation, DEADBAND);
      }      
    } else {
      driveTrain.arcadeDrive(-throttle, rotation, DEADBAND); 
    }

  

    

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

  public double limitOutput(double number, double maxOutput) {
    if (number > 1.0) {
        number = 1.0;
    }
    if (number < -1.0) {
        number = -1.0;
    }

    if (number > maxOutput) {
        return maxOutput;
    }
    if (number < -maxOutput) {
        return -maxOutput;
    }

    return number;
}

}
