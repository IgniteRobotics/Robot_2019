/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivetrain extends Subsystem {

  private WPI_TalonSRX leftMaster;
  private WPI_VictorSPX leftFollower;
  private WPI_TalonSRX rightMaster;
  private WPI_VictorSPX rightFollower;
  
  private DifferentialDrive drive;

  private Command defaultCommand;

  public Drivetrain (int leftMasterID, int leftFollowerID, int rightMasterID, int rightFollowerID) {

    leftMaster = new WPI_TalonSRX(leftMasterID);
    leftFollower = new WPI_VictorSPX(leftFollowerID);
    rightMaster = new WPI_TalonSRX(rightMasterID);
    rightFollower = new WPI_VictorSPX(rightFollowerID);

    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster); 

    // leftMaster.setInverted(true);
    // leftFollower.setInverted(InvertType.FollowMaster); //TODO: set me

    // rightMaster.setInverted(true);
    // rightFollower.setInverted(InvertType.FollowMaster);

  }
  public void arcadeDrive(double power, double rotation){
    drive.arcadeDrive(power, rotation, true);
  }

  public void setDefault(Command command){
    defaultCommand = command;
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(this.defaultCommand);  
  }

}
