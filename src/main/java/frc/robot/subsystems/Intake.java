/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IgniteSubsystem;

import badlog.lib.*;

public class Intake extends IgniteSubsystem {

  private WPI_TalonSRX intakeMotor;
  private DoubleSolenoid intake;
  private DigitalInput intakeBeamBreak;

  private boolean intakeState;

  private Command defaultCommand;

  public Intake(int pcmID, int intakeMotorID, int intakeSolenoidOpen, int intakeSolenoidClose, int intakeBeamBreakID) {

    intakeMotor = new WPI_TalonSRX(intakeMotorID);
    intake = new DoubleSolenoid(pcmID, intakeSolenoidOpen, intakeSolenoidClose);
    intakeBeamBreak = new DigitalInput(intakeBeamBreakID);

    intakeMotor.setNeutralMode(NeutralMode.Brake);

    intakeMotor.setInverted(true);

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
    BadLog.createTopic("Intake/Percent Output", BadLog.UNITLESS, () -> this.getPercentOutput(), "hide",
        "join:Intake/Output percents");
    BadLog.createTopic("Intake/Voltage", "V", () -> this.getVoltage(), "hide", "join:Intake/Output voltage");
    BadLog.createTopic("Intake/Current", "A", () -> this.getCurrent(), "hide", "join:Intake/Output current");
  }

  public void outputTelemetry() {
    SmartDashboard.putNumber("Intake/Voltage", this.getVoltage());
    SmartDashboard.putNumber("Intake/Current", this.getCurrent());
    SmartDashboard.putNumber("Intake/Percent out", this.getPercentOutput());
    SmartDashboard.putBoolean("Intake/Is intake open?", this.isIntakeOpen());
    SmartDashboard.putBoolean("Intake/Has cargo?", this.hasCargo());
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(this.defaultCommand);
  }

  public void setOpenLoop(double power) {
    intakeMotor.set(ControlMode.PercentOutput, power);
  }

  public double getVoltage() {
    return intakeMotor.getMotorOutputVoltage();
  }

  public double getCurrent() {
    return intakeMotor.getOutputCurrent();
  }

  public double getPercentOutput() {
    return intakeMotor.getMotorOutputPercent();
  }

  public void stopIntakeMotor() {
    intakeMotor.stopMotor();
  }

  public void openIntake() {
    intake.set(DoubleSolenoid.Value.kForward);
    intakeState = true;
  }

  public void closeIntake() {
    intake.set(DoubleSolenoid.Value.kReverse);
    intakeState = false;
  }

  public boolean isIntakeOpen() {
    return intakeState;
  }

  public boolean hasCargo() {
    return !intakeBeamBreak.get();
  }

}
