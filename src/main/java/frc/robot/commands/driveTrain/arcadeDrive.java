/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.driveTrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    SmartDashboard.putNumber("VisionDrive/Max Power", -0.4);
    SmartDashboard.putNumber("VisionDrive/Angle Tolerance", 0.5);

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
    SmartDashboard.putNumber("VisionDrive/Throttle", throttle);
    double maxPower = -0.55; //SmartDashboard.getNumber("VisionDrive/Max Power", );
    double angleTolerance = 2.0; // SmartDashboard.getNumber("VisionDrive/Angle Tolerance");

    
    if (driverJoystick.getRawButton(1)) { //button pushed
      double targetAngle = 0;
      targetAngle = jetson.getTargetAngle(); //jetson.getDirectTurn();
      if (throttle < maxPower)
          throttle = maxPower; 
      if (targetAngle != 0) { // got a value in NT
        if (throttle > 0) {
            targetAngle = 0;
        }
        if (targetAngle >= -angleTolerance && targetAngle <= angleTolerance)
          targetAngle = 0;
        double angle = limitOutput((kP * targetAngle)/35, 0.55); 
        
        driveTrain.arcadeDrive(-throttle, angle, DEADBAND);
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
