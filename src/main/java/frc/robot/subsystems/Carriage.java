/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import badlog.lib.BadLog;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IgniteSubsystem;
import frc.robot.util.LogUtil;

public class Carriage extends IgniteSubsystem {

  private Solenoid cargoEject;
  private Solenoid beak;
  private DigitalInput beamBreak;

  private boolean cargoEjectState;
  private boolean beakState;

  private Command defaultCommand;

  public Carriage(int pcmID, int cargoEjectSolenoid, int beakSolenoid, int beamBreakID) {

    cargoEject = new Solenoid(pcmID, cargoEjectSolenoid);
    beak = new Solenoid(pcmID, beakSolenoid);
    beamBreak = new DigitalInput(beamBreakID);

  }
  
  public void establishDefaultCommand(Command command) {
    this.defaultCommand = command;
    initDefaultCommand();
  }

  public boolean checkSystem() {
    return true;
  }

  public void writeToLog() {
    BadLog.createTopicStr("Carriage/Is cargo eject open?", "bool", () -> LogUtil.fromBool(this.isCargoEjectOpen()));
    BadLog.createTopicStr("Carriage/Is beak open?", "bool", () -> LogUtil.fromBool(this.isBeakOpen()));
    BadLog.createTopicStr("Carriage/Has cargo?", "bool", () -> LogUtil.fromBool(this.hasCargo()));
  }

  public void outputTelemetry() {
    SmartDashboard.putBoolean("Carriage/Is cargo eject open?", this.isCargoEjectOpen());
    SmartDashboard.putBoolean("Carriage/Is beak open?", this.isBeakOpen());
    SmartDashboard.putBoolean("Carriage/Has cargo?", this.hasCargo());
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(this.defaultCommand);  
  }

  private void pollCargoEject() {
    cargoEjectState = cargoEject.get();
  }

  private void pollBeak() {
    beakState = beak.get();
  }

  public boolean isCargoEjectOpen() {
    pollCargoEject();
    return cargoEjectState;
  }

  public boolean isBeakOpen() {
    pollBeak();
    return beakState;
  }

  public void ejectCargo() {
    cargoEject.set(true);
  }

  public void toggleBeak() {
    if (isBeakOpen()) {
      retractBeak();
    } else {
      openBeak();
    }
  }

  public void openBeak() {
    beak.set(true);
  }

  public void retractEjectCargo() {
    cargoEject.set(false);
  }

  public void retractBeak() {
    beak.set(false);
  }

  public boolean hasCargo() {
    return !beamBreak.get();
  }

}
