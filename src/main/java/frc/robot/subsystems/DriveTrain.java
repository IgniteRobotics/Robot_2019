/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.subsystems.IgniteSubsystem;
import frc.robot.Util;

public class DriveTrain extends IgniteSubsystem {

  private WPI_TalonSRX leftMaster;
  private WPI_VictorSPX leftFollower;
  private WPI_TalonSRX rightMaster;
  private WPI_VictorSPX rightFollower;
  
  private PIDController turnController;

  private AHRS navX;

  private Command defaultCommand;

  private final double kP_TURN = 0;
  private final double kI_TURN = 0;
  private final double kD_TURN = 0;

  private final double kP_DRIVE = 0;
  private final double kI_DRIVE = 0;
  private final double kD_DRIVE = 0;
  private final double kF_DRIVE = 0;

  private final int CRUISE_VELOCITY = 0;
  private final int MAX_ACCELERATION = 0;

  private final double TURN_TOLERANCE = 2.0f;
  private final double DRIVE_TOLERANCE = 100.0f;

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

    leftMaster.setInverted(true);
    // leftMaster.setSensorPhase(false);

    leftFollower.setInverted(InvertType.FollowMaster);
    rightFollower.setInverted(InvertType.FollowMaster);

    rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 20);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10, 20);

    leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 20);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10, 20);

    leftMaster.selectProfileSlot(1, 0);
    leftMaster.config_kF(0, kF_DRIVE, 10);
    leftMaster.config_kP(0, kP_DRIVE, 10);
    leftMaster.config_kI(0, kI_DRIVE, 10);
    leftMaster.config_kD(0, kD_DRIVE, 10);
    
    leftMaster.configMotionCruiseVelocity(CRUISE_VELOCITY, 10);
    leftMaster.configMotionAcceleration(MAX_ACCELERATION, 10);

    turnController = new PIDController(kP_TURN, kI_TURN, kD_TURN, navX, new SpeedControllerGroup(leftMaster, rightMaster));

    turnController.setInputRange(-180.0f,  180.0f);
    turnController.setOutputRange(-1.0, 1.0);
    turnController.setAbsoluteTolerance(TURN_TOLERANCE);
    turnController.setContinuous(true);

  }

  public void establishDefaultCommand(Command command) {
    this.defaultCommand = command;
    initDefaultCommand();
  }

  public boolean checkSystem() {
    return true;
  }

  public void writeToLog() {
  }

  public void outputTelemetry() {
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(this.defaultCommand);  
  }
  
  public void arcadeDrive(double throttle, double rotation, double deadband) {

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

		throttle = limit(throttle);
		throttle = Util.applyDeadband(throttle, deadband);

		rotation = limit(rotation);
		rotation = Util.applyDeadband(rotation, deadband);

		setOpenLoopLeft(leftMotorOutput);
    setOpenLoopRight(rightMotorOutput);
    
  }

  public void setOpenLoopLeft(double power) {
    leftMaster.set(ControlMode.PercentOutput, power);
  }

  public void setOpenLoopRight(double power) {
    rightMaster.set(ControlMode.PercentOutput, power);
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

  public double getLeftVoltage() {
    return leftMaster.getMotorOutputVoltage();
  }

  public void setMotionMagicPosition(double position) {
    leftMaster.set(ControlMode.MotionMagic, position);
    rightMaster.set(ControlMode.MotionMagic, position);
  }

  public boolean isMotionMagicDone() {
    return Math.abs(this.getLeftEncoderPos() - leftMaster.getClosedLoopError()) < DRIVE_TOLERANCE;
  }

  public void turnToAngle(double setpoint) {
    turnController.setSetpoint(setpoint);
  }

  public boolean isTurnCompleted() {
    return Math.abs(turnController.getError() - this.getAngle()) <= TURN_TOLERANCE;
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
  
  public void zeroSensors() {
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
