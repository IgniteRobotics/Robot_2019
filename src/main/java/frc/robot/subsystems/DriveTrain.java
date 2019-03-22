/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IgniteSubsystem;
import frc.robot.util.Util;

import badlog.lib.*;

public class DriveTrain extends IgniteSubsystem implements PIDOutput {

  private WPI_TalonSRX leftMaster;
  private WPI_VictorSPX leftFollower;
  private WPI_TalonSRX rightMaster;
  private WPI_VictorSPX rightFollower;
  
  private PIDController turnController;

  private AHRS navX;

  private Command defaultCommand;

  private final double kP_TURN = 0.01;
  private final double kI_TURN = 0;
  private final double kD_TURN = 0.009;

  private final double kP_DRIVE = 1;
  private final double kI_DRIVE = 0;
  private final double kD_DRIVE = 0;
  private final double kF_DRIVE = 0;

  private final int CRUISE_VELOCITY = 2000;
  private final int MAX_ACCELERATION = 1000;

  private final double TURN_TOLERANCE = 2.0f;
  private final double DRIVE_TOLERANCE = 100.0;

  private double rotateToAngleRate;

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

    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster); 

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    leftFollower.setInverted(InvertType.FollowMaster);
    rightFollower.setInverted(InvertType.FollowMaster);

    rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 20);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10, 20);

    leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 20);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10, 20);

    leftMaster.selectProfileSlot(2, 0);
    leftMaster.config_kF(2, kF_DRIVE, 10);
    leftMaster.config_kP(2, kP_DRIVE, 10);
    leftMaster.config_kI(2, kI_DRIVE, 10);
    leftMaster.config_kD(2, kD_DRIVE, 10);

    rightMaster.selectProfileSlot(1, 0);
    rightMaster.config_kF(1, kF_DRIVE, 10);
    rightMaster.config_kP(1, kP_DRIVE, 10);
    rightMaster.config_kI(1, kI_DRIVE, 10);
    rightMaster.config_kD(1, kD_DRIVE, 10);
    
    leftMaster.configMotionCruiseVelocity(CRUISE_VELOCITY, 10);
    leftMaster.configMotionAcceleration(MAX_ACCELERATION, 10);

    rightMaster.configMotionCruiseVelocity(CRUISE_VELOCITY, 10);
    rightMaster.configMotionAcceleration(MAX_ACCELERATION, 10);

    turnController = new PIDController(kP_TURN, kI_TURN, kD_TURN, navX, this);

    turnController.setInputRange(-180.0f,  180.0f);
    turnController.setOutputRange(-1.0, 1.0);
    turnController.setAbsoluteTolerance(TURN_TOLERANCE);
    turnController.setContinuous(true);

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
    SmartDashboard.putNumber("Drivetrain/Left enc pos", this.getLeftEncoderPos());
    SmartDashboard.putNumber("Drivetrain/Right enc pos", this.getRightEncoderPos());
    SmartDashboard.putNumber("Drivetrain/Left enc vel", this.getLeftEncoderVel());
    SmartDashboard.putNumber("Drivetrain/Right enc vel", this.getRightEncoderVel());
    SmartDashboard.putNumber("Drivetrain/Left master voltage", this.getLeftMasterVoltage());
    SmartDashboard.putNumber("Drivetrain/Right master voltage", this.getRightMasterVoltage());
    SmartDashboard.putNumber("Drivetrain/Left follower voltage", this.getLeftFollowerVoltage());
    SmartDashboard.putNumber("Drivetrain/Right follower voltage", this.getRightFollowerVoltage());
    SmartDashboard.putNumber("Drivetrain/Left master current", this.getLeftMasterCurrent());
    SmartDashboard.putNumber("Drivetrain/Right master current", this.getRightMasterCurrent());
    SmartDashboard.putNumber("Drivetrain/Left percent out", this.getLeftPercentOutput());
    SmartDashboard.putNumber("Drivetrain/Right percent out", this.getRightPercentOutput());
    SmartDashboard.putBoolean("Drivetrain/Is navX connected?", this.isConnected());
    SmartDashboard.putNumber("Drivetrain/Angle", this.getAngle());
    SmartDashboard.putNumber("Drivetrain/Yaw", this.getYaw());
    SmartDashboard.putNumber("Drivetrain/Closed loop target", this.getClosedLoopTarget());
    SmartDashboard.putNumber("Drivetrain/Turn error", this.getTurnError());
    SmartDashboard.putNumber("Drivetrain/Turn setpoint", this.getTurnSetpoint());
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(this.defaultCommand);  
  }
  
  public void arcadeDrive(double throttle, double rotation, double deadband) {

		throttle = limit(throttle);
		throttle = Util.applyDeadband(throttle, deadband);

		rotation = limit(rotation);
		rotation = Util.applyDeadband(rotation, deadband);

    throttle = Math.copySign(throttle * throttle, throttle);
    rotation = Math.copySign(rotation * rotation, rotation);

		double leftMotorOutput;
		double rightMotorOutput;

		double maxInput = Math.copySign(Math.max(Math.abs(throttle), Math.abs(rotation)), throttle);

		if (throttle >= 0.0) {
			// First quadrant, else second quadrant
			if (rotation >= 0.0) {
				leftMotorOutput = maxInput;
				rightMotorOutput = throttle - rotation;
			} else {
				leftMotorOutput = throttle + rotation;
				rightMotorOutput = maxInput;
			}
		} else {
			// Third quadrant, else fourth quadrant
			if (rotation >= 0.0) {
				leftMotorOutput = throttle + rotation;
				rightMotorOutput = maxInput;
			} else {
				leftMotorOutput = maxInput;
				rightMotorOutput = throttle - rotation;
			}
		}

		setOpenLoopLeft(limit(leftMotorOutput));
    setOpenLoopRight(limit(rightMotorOutput));
    
  }

  public void setOpenLoopLeft(double power) {
    leftMaster.set(ControlMode.PercentOutput, power);
  }

  public void setOpenLoopRight(double power) {
    rightMaster.set(ControlMode.PercentOutput, power);
  }

  public int getLeftEncoderPos() {
    return leftMaster.getSelectedSensorPosition();
  }

  public int getRightEncoderPos() {
    return rightMaster.getSelectedSensorPosition();
  }

  public double getLeftEncoderVel() {
    return leftMaster.getSelectedSensorVelocity();
  }

  public double getRightEncoderVel() {
    return rightMaster.getSelectedSensorVelocity();
  }

  public double getLeftMasterVoltage() {
    return leftMaster.getMotorOutputVoltage();
  }

  public void setMotionMagicPosition(double position_inches) {
    double ticks = Util.getEncoderTicksFromInches(position_inches);
    leftMaster.set(ControlMode.MotionMagic, ticks);
    rightMaster.set(ControlMode.MotionMagic, ticks);
  }

  public boolean isMotionMagicDone() {
    return Math.abs(this.getClosedLoopTarget() - this.getLeftEncoderPos()) < DRIVE_TOLERANCE;
  }

  public double getClosedLoopTarget() {
    return leftMaster.getClosedLoopTarget();
  }

  @Override
  public void pidWrite(double output) {
    rotateToAngleRate = output;
  }

  public void enableTurnController(double setpoint) {
    turnController.setSetpoint(setpoint);
    rotateToAngleRate = 0;
    turnController.enable();
  }

  public void turnToAngle() {
    this.setOpenLoopLeft(rotateToAngleRate);
    this.setOpenLoopRight(-rotateToAngleRate);
  }

  public boolean isTurnCompleted() {
    return turnController.onTarget();
  }

  public void stopTurnController() {
    rotateToAngleRate = 0;
    turnController.disable();
    turnController.reset();
  }

  public double getTurnError() {
    return turnController.getError();
  }

  public double getTurnSetpoint() {
    return turnController.getSetpoint();
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
    zeroAngle();
    zeroEncoders();
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

  public void zeroEncoders() {
    rightMaster.setSelectedSensorPosition(0);
    leftMaster.setSelectedSensorPosition(0);
  }

  public boolean isConnected() {
    return navX.isConnected();
  }

	private double limit(double value) {
		if (value > 1.0) {
			return 1.0;
		}
		if (value < -1.0) {
			return -1.0;
		}
		return value;
	}

}
