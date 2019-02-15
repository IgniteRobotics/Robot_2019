/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IgniteSubsystem;

import badlog.lib.*;

public class DriveTrain extends IgniteSubsystem {

  private WPI_TalonSRX leftMaster;
  private WPI_VictorSPX leftFollower;
  private WPI_TalonSRX rightMaster;
  private WPI_VictorSPX rightFollower;
  
  private DifferentialDrive drive;
  
  private AHRS navX;

  private Command defaultCommand;

  public DriveTrain(int leftMasterID, int leftFollowerID, int rightMasterID, int rightFollowerID) {

    leftMaster = new WPI_TalonSRX(leftMasterID);
    leftFollower = new WPI_VictorSPX(leftFollowerID);
    rightMaster = new WPI_TalonSRX(rightMasterID);
    rightFollower = new WPI_VictorSPX(rightFollowerID);

    navX = new AHRS(SPI.Port.kMXP, (byte)200);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);

    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    drive = new DifferentialDrive(leftMaster, rightMaster);

    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster); 

    // leftMaster.setInverted(true);
    // leftFollower.setInverted(InvertType.FollowMaster); //TODO: set me
    // leftMaster.setSensorPhase(false);

    // rightMaster.setInverted(true);
    // rightFollower.setInverted(InvertType.FollowMaster);
    // rightMaster.setSensorPhase(false);

    writeToLog();

  }

  public void establishDefaultCommand(Command command) {
    this.defaultCommand = command;
    initDefaultCommand();
  }

  public boolean checkSystem() {
    return true;
  }

  public void writeToLog() {
    BadLog.createTopic("Drivetrain/Right Percent Output", BadLog.UNITLESS, () -> this.getRightPercentOutput(), "hide", "join:Drivetrain/Output percents");
    BadLog.createTopic("Drivetrain/Left Percent Output", BadLog.UNITLESS, () -> this.getLeftPercentOutput(), "hide", "join:Drivetrain/Output percents");
    BadLog.createTopic("Drivetrain/Right Master Current", "A", () -> this.getRightMasterCurrent(), "hide", "join:Drivetrain/Output Currents");
    BadLog.createTopic("Drivetrain/Left Master Current", "A", () -> this.getLeftMasterCurrent(), "hide", "join:Drivetrain/Output Currents");
    BadLog.createTopic("Drivetrain/Right Master Voltage", "V", () -> this.getRightMasterVoltage(), "hide", "join:Drivetrain/Output voltages");
    BadLog.createTopic("Drivetrain/Left Master Voltage", "V", () -> this.getLeftMasterVoltage(), "hide", "join:Drivetrain/Output voltages");
    BadLog.createTopic("Drivetrain/Right Follower Voltage", "V", () -> this.getRightMasterVoltage(), "hide", "join:Drivetrain/Output voltages");
    BadLog.createTopic("Drivetrain/Left Follower Voltage", "V", () -> this.getLeftMasterVoltage(), "hide", "join:Drivetrain/Output voltages");
  }

  public void outputTelemetry() {
    SmartDashboard.putNumber("Left enc pos", this.getLeftEncoderPos());
    SmartDashboard.putNumber("Right enc pos", this.getRightEncoderPos());
    SmartDashboard.putNumber("Left enc vel", this.getLeftEncoderVel());
    SmartDashboard.putNumber("Right enc vel", this.getRightEncoderVel());
    SmartDashboard.putNumber("Left master voltage", this.getLeftMasterVoltage());
    SmartDashboard.putNumber("Right master voltage", this.getRightMasterVoltage());
    SmartDashboard.putNumber("Left follower voltage", this.getLeftFollowerVoltage());
    SmartDashboard.putNumber("Right follower voltage", this.getRightFollowerVoltage());
    SmartDashboard.putNumber("Left percent out", this.getLeftPercentOutput());
    SmartDashboard.putNumber("Right percent out", this.getRightPercentOutput());
    SmartDashboard.putBoolean("Is navX connected?", this.isConnected());
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(this.defaultCommand);  
  }
  
  public void arcadeDrive(double power, double rotation){
    drive.arcadeDrive(power, rotation, true);
  }

  public int getLeftEncoderPos() {
    return leftMaster.getSensorCollection().getQuadraturePosition();
  }

  public int getRightEncoderPos() {
    return rightMaster.getSensorCollection().getQuadraturePosition();
  }

  public double getLeftEncoderVel() {
    return leftMaster.getSensorCollection().getQuadratureVelocity();
  }

  public double getRightEncoderVel() {
    return rightMaster.getSensorCollection().getQuadratureVelocity();
  }

  public double getLeftMasterVoltage() {
    return leftMaster.getMotorOutputVoltage();
  }
  
  public double getRightMasterVoltage() {
    return rightMaster.getMotorOutputVoltage();
  }
  
  public double getLeftFollowerVoltage() {
    return leftFollower.getMotorOutputVoltage();
  }
  
  public double getRightFollowerVoltage() {
    return rightFollower.getMotorOutputVoltage();
  }

  public double getLeftPercentOutput() {
    return leftMaster.getMotorOutputPercent();
  }
  
  public double getRightPercentOutput() {
    return rightMaster.getMotorOutputPercent();
  }

  public double getLeftMasterCurrent() {
    return leftMaster.getOutputCurrent();
  }

  public double getRightMasterCurrent() {
    return rightMaster.getOutputCurrent();
  }

  public void zeroSensors() {
    rightMaster.getSensorCollection().setQuadraturePosition(0, 10);
    leftMaster.getSensorCollection().setQuadraturePosition(0, 10);
    zeroAngle();
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
