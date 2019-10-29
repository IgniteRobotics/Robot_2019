/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import badlog.lib.*;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.InvertType;

import frc.robot.subsystems.IgniteSubsystem;
import frc.robot.commands.elevator.ElevatorState;
import frc.robot.util.Util;
import frc.robot.util.LogUtil;
import frc.robot.Constants;

public class Elevator extends IgniteSubsystem {

  private WPI_TalonSRX elevatorMaster;
  private WPI_VictorSPX elevatorFollower;

  private Command defaultCommand;

  private final double kF = 0;
  private final double kP = 1;
  private final double kI = 0;
  private final double kD = 0;

  private final int MAX_ACCELERATION = 8000 / 2;
  private final int CRUISE_VELOCITY = 6000;

  private final int TOLERANCE = 200;

  private ElevatorState state;

  private HashMap<ElevatorState, Integer> hatchSetpoints = new HashMap<ElevatorState, Integer>();
  private HashMap<ElevatorState, Integer> cargoSetpoints = new HashMap<ElevatorState, Integer>();

  public Elevator(int elevatorMasterID, int elevatorFollowerID) {

    this.setState(ElevatorState.Zero);

    elevatorMaster = new WPI_TalonSRX(elevatorMasterID);
    elevatorFollower = new WPI_VictorSPX(elevatorFollowerID);

    elevatorMaster.setNeutralMode(NeutralMode.Brake);
    elevatorFollower.setNeutralMode(NeutralMode.Brake);

    elevatorFollower.follow(elevatorMaster);

    elevatorMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    elevatorMaster.setSensorPhase(true);
    elevatorMaster.setInverted(false);

    elevatorFollower.setInverted(InvertType.FollowMaster);

    elevatorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 20);
    elevatorMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10, 20);

    elevatorMaster.selectProfileSlot(0, 0);
    elevatorMaster.config_kF(0, kF, 10);
    elevatorMaster.config_kP(0, kP, 10);
    elevatorMaster.config_kI(0, kI, 10);
    elevatorMaster.config_kD(0, kD, 10);

    elevatorMaster.configMotionCruiseVelocity(CRUISE_VELOCITY, 10);
    elevatorMaster.configMotionAcceleration(MAX_ACCELERATION, 10);

    populateSetpoints();

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
    BadLog.createTopic("Elevator/Master Percent Output", BadLog.UNITLESS, () -> this.getPercentOutput(), "hide",
        "join:Elevator/Output percents");
    BadLog.createTopic("Elevator/Master Voltage", "V", () -> this.getMasterVoltage(), "hide",
        "join:Elevator/Output Voltages");
    BadLog.createTopic("Elevator/Follower Voltage", "V", () -> this.getFollowerVoltage(), "hide",
        "join:Elevator/Output Voltages");
    BadLog.createTopic("Elevator/Master Current", "A", () -> this.getMasterCurrent(), "hide",
        "join:Elevator/Output Current");
    BadLog.createTopic("Elevator/Position", "ticks", () -> (double) this.getEncoderPos(), "hide",
        "join:Elevator/Position");
    BadLog.createTopic("Elevator/Velocity", "ticks", () -> (double) this.getEncoderVel(), "hide",
        "join:Elevator/Velocity");
    BadLog.createTopicStr("Elevator/Fwd limit", "bool", () -> LogUtil.fromBool(this.isFwdLimitTripped()));
    BadLog.createTopicStr("Elevator/Rev limit", "bool", () -> LogUtil.fromBool(this.isRevLimitTripped()));
  }

  public void outputTelemetry() {
    SmartDashboard.putString("Elevator/State", this.getState().toString());
    SmartDashboard.putNumber("Elevator/Pos", this.getEncoderPos());
    SmartDashboard.putNumber("Elevator/Vel", this.getEncoderVel());
    SmartDashboard.putNumber("Elevator/Master voltage", this.getMasterVoltage());
    SmartDashboard.putNumber("Elevator/Follower voltage", this.getFollowerVoltage());
    SmartDashboard.putNumber("Elevator/Master current", this.getMasterCurrent());
    SmartDashboard.putNumber("Elevator/Percent out", this.getPercentOutput());
    SmartDashboard.putBoolean("Elevator/Is fwd limit switch tripped?", this.isFwdLimitTripped());
    SmartDashboard.putBoolean("Elevator/Is rev limit switch tripped?", this.isRevLimitTripped());
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(this.defaultCommand);
  }

  public void setOpenLoop(double percentage) {
    elevatorMaster.set(ControlMode.PercentOutput, percentage);
  }

  public void setOpenLoop(double percentage, double deadband) {
    percentage = Util.applyDeadband(percentage, Constants.ELEVATOR_JOG_DEADBAND);
    setOpenLoop(percentage);
  }

  public void setMotionMagicPosition(double position) {
    elevatorMaster.set(ControlMode.MotionMagic, position);
  }

  public boolean isMotionMagicDone() {
    return Math.abs(elevatorMaster.getClosedLoopTarget() - this.getEncoderPos()) <= TOLERANCE;
  }

  public int getEncoderPos() {
    return elevatorMaster.getSelectedSensorPosition();
  }

  public double getEncoderVel() {
    return elevatorMaster.getSelectedSensorVelocity();
  }

  public double getMasterVoltage() {
    return elevatorMaster.getMotorOutputVoltage();
  }

  public double getFollowerVoltage() {
    return elevatorFollower.getMotorOutputVoltage();
  }

  public double getPercentOutput() {
    return elevatorMaster.getMotorOutputPercent();
  }

  public double getMasterCurrent() {
    return elevatorMaster.getOutputCurrent();
  }

  public void zeroSensors() {
    elevatorMaster.setSelectedSensorPosition(0);
  }

  public boolean isFwdLimitTripped() {
    return elevatorMaster.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean isRevLimitTripped() {
    return elevatorMaster.getSensorCollection().isRevLimitSwitchClosed();
  }

  public void stop() {
    elevatorMaster.stopMotor();
  }

  public int getSetpoint(ElevatorState state, boolean hasCargo) {
    setState(state);
    if (hasCargo) {
      return cargoSetpoints.get(state);
    } else {
      return hatchSetpoints.get(state);
    }
  }

  public void setState(ElevatorState state) {
    this.state = state;
  }

  public ElevatorState getState() {
    return state;
  }

  private void populateSetpoints() {
    cargoSetpoints.put(ElevatorState.Level1, Constants.ROCKET_CARGO_L1);
    cargoSetpoints.put(ElevatorState.Level2, Constants.ROCKET_CARGO_L2);
    cargoSetpoints.put(ElevatorState.Level3, Constants.ROCKET_CARGO_L3);
    cargoSetpoints.put(ElevatorState.Zero, 0);
    cargoSetpoints.put(ElevatorState.CargoShipCargo, Constants.CARGO_SHIP_CARGO);
    cargoSetpoints.put(ElevatorState.HatchPickup, Constants.HATCH_PICKUP);
    hatchSetpoints.put(ElevatorState.Level1, Constants.ROCKET_HATCH_L1);
    hatchSetpoints.put(ElevatorState.Level2, Constants.ROCKET_HATCH_L2);
    hatchSetpoints.put(ElevatorState.Level3, Constants.ROCKET_HATCH_L3);
    hatchSetpoints.put(ElevatorState.CargoShipCargo, Constants.CARGO_SHIP_CARGO);
    hatchSetpoints.put(ElevatorState.Zero, 0);
    hatchSetpoints.put(ElevatorState.HatchPickup, Constants.HATCH_PICKUP);
  }

}
