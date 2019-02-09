/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public static final int pcmID = 0;
  
  // Carriage
  public static final int cargoEjectSolenoid = 0;
  public static final int beakSolenoid = 0;
  public static final int hatchEjectSolenoid = 0;
  public static final int beamBreakID = 0;

  // Drivetrain
  public static final int leftMasterID = 0;
  public static final int leftFollowerID = 0;
  public static final int rightMasterID = 0;
  public static final int rightFollowID= 0;

  // Elevator
  public static final int elevatorMasterID = 0;
  public static final int elevatorFollowerID = 0;

  // Intake
  public static final int intakeSolenoid = 0;
  public static final int intakeMotorID = 0;

}
