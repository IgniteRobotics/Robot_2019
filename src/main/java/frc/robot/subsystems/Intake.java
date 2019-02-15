/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IgniteSubsystem;

import badlog.lib.*;

public class Intake extends IgniteSubsystem {
  
  private WPI_VictorSPX intakeMotor;
  private Solenoid intake;

  private boolean intakeState;

  private Command defaultCommand;

  public Intake(int pcmID, int intakeMotorID, int intakeSolenoid) {

    intakeMotor = new WPI_VictorSPX(intakeMotorID);
    intake = new Solenoid(pcmID, intakeSolenoid);

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
    BadLog.createTopic("Intake/Percent Output", BadLog.UNITLESS, () -> this.getPercentOutput(), "hide", "join:Intake/Output percents");
    BadLog.createTopic("Intake/Voltage", "V", () -> this.getVoltage(), "hide", "join:Intake/Output voltage");
  }

  public void outputTelemetry() {
    SmartDashboard.putNumber("Voltage", this.getVoltage());
    SmartDashboard.putNumber("Percent out", this.getPercentOutput());
    SmartDashboard.putBoolean("Is intake open?", this.isIntakeOpen());
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

  public double getPercentOutput() {
    return intakeMotor.getMotorOutputPercent();
  }

  public void stopIntakeMotor() {
    intakeMotor.stopMotor();
  }

  private void pollIntakeSolenoid() {
    intakeState = intake.get();
  }

  public void openIntake() {
    intake.set(true);
  }
  
  public void closeIntake() {
    intake.set(false);
  }

  public boolean isIntakeOpen() {
    pollIntakeSolenoid();
    return intakeState;
  }
  
}
