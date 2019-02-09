/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivetrain extends Subsystem {

  private WPI_TalonSRX leftMaster;
  private WPI_VictorSPX leftFollower;
  private WPI_TalonSRX rightMaster;
  private WPI_VictorSPX rightFollower;
  
  private DifferentialDrive drive;
  
  private AHRS navX;

  private Command defaultCommand;

  public Drivetrain (int leftMasterID, int leftFollowerID, int rightMasterID, int rightFollowerID) {

    leftMaster = new WPI_TalonSRX(leftMasterID);
    leftFollower = new WPI_VictorSPX(leftFollowerID);
    rightMaster = new WPI_TalonSRX(rightMasterID);
    rightFollower = new WPI_VictorSPX(rightFollowerID);

    navX = new AHRS(SPI.Port.kMXP, (byte)200);

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

  public void setCommandDefault(Command command){
    this.defaultCommand = command;
    initDefaultCommand();
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(this.defaultCommand);  
  }

  public int getLeftEncoderPos() {
    return leftMaster.getSensorCollection().getQuadraturePosition();
  }

  public int getRightEncoder() {
    return rightMaster.getSensorCollection().getQuadraturePosition();
  }

  public double getLeftEncoderVel() {
    return leftMaster.getSensorCollection().getQuadratureVelocity();
  }

  public double getRightEncoderVel() {
    return rightMaster.getSensorCollection().getQuadratureVelocity();
  }

  public double getLeftVoltage() {
    return leftMaster.getMotorOutputVoltage();
  }
  
  public double getRightVoltage() {
    return rightMaster.getMotorOutputVoltage();
  }
  
  public double getLeftPercentOutput() {
    return leftMaster.getMotorOutputPercent();
  }
  
  public double getRightPercentOutput() {
    return rightMaster.getMotorOutputPercent();
  }
  
  public void zeroEncoders() {
    rightMaster.getSensorCollection().setQuadraturePosition(0, 10);
    leftMaster.getSensorCollection().setQuadraturePosition(0, 10);
  }
  
  public void stop() {
    leftMaster.stopMotor();
    rightMaster.stopMotor();
  }

  public double getAngle() {
    return navX.getAngle();
  }

  public double getYaw() {
    return navX.getYaw();
  }

  public void zeroAngle() {
    navX.reset();
  }

  public boolean isConnected() {
    return navX.isConnected();
  }

}
