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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.subsystems.IgniteSubsystem;

public class DriveTrain extends IgniteSubsystem {

  private WPI_TalonSRX leftMaster;
  private WPI_VictorSPX leftFollower;
  private WPI_TalonSRX rightMaster;
  private WPI_VictorSPX rightFollower;
  
  private DifferentialDrive drive;
  
  private PIDController turnController;

  private AHRS navX;

  private Command defaultCommand;

  private final double kP_TURN = 0;
  private final double kI_TURN = 0;
  private final double kD_TURN = 0;

  private final double TURN_TOLERANCE = 2.0f;

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

    // leftMaster.setInverted(true); //TODO: set me
    // leftMaster.setSensorPhase(false);

    // rightMaster.setInverted(true);
    // rightMaster.setSensorPhase(false);

    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster); 

    leftFollower.setInverted(InvertType.FollowMaster);
    rightFollower.setInverted(InvertType.FollowMaster);

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

		double maxInput = Math.copySign(Math.max(Math.abs(throttle), Math.abs(rotation)), rotation);

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
		rotation = applyDeadband(rotation, deadband);

		rotation = limit(rotation);
		rotation = applyDeadband(rotation, deadband);

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

	private double applyDeadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0.0;
		}
	}


}
