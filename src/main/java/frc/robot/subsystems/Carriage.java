/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import java.util.HashMap;

import badlog.lib.BadLog;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.CarriageLevel;
import frc.robot.subsystems.IgniteSubsystem;
import frc.robot.util.LogUtil;

public class Carriage extends IgniteSubsystem {

  private Solenoid cargoEject;
  private Solenoid beak;
  private DigitalInput beamBreak;
  private DigitalInput hatchLimitSwitch;

  private boolean cargoEjectState;
  private boolean beakState;

  private Command defaultCommand;

  private HashMap<CarriageLevel, Integer> hatchSetpoints = new HashMap<CarriageLevel, Integer>();
  private HashMap<CarriageLevel, Integer> cargoSetpoints = new HashMap<CarriageLevel, Integer>();

  public Carriage(int pcmID, int cargoEjectSolenoid, int beakSolenoid, int beamBreakID, int hatchLimitSwitchID) {

    cargoEject = new Solenoid(pcmID, cargoEjectSolenoid);
    beak = new Solenoid(pcmID, beakSolenoid);
    beamBreak = new DigitalInput(beamBreakID);
    hatchLimitSwitch = new DigitalInput(hatchLimitSwitchID);

    cargoSetpoints.put(CarriageLevel.Level1, Constants.ROCKET_CARGO_L1);
    cargoSetpoints.put(CarriageLevel.Level2, Constants.ROCKET_CARGO_L2);
    cargoSetpoints.put(CarriageLevel.Level3, Constants.ROCKET_CARGO_L3);
    cargoSetpoints.put(CarriageLevel.Zero, 0);
    cargoSetpoints.put(CarriageLevel.CargoShipCargo, Constants.CARGO_SHIP_CARGO);
    cargoSetpoints.put(CarriageLevel.HatchPickup, Constants.HATCH_PICKUP);
    hatchSetpoints.put(CarriageLevel.Level1, Constants.ROCKET_HATCH_L1);
    hatchSetpoints.put(CarriageLevel.Level2, Constants.ROCKET_HATCH_L2);
    hatchSetpoints.put(CarriageLevel.Level3, Constants.ROCKET_HATCH_L3);
    hatchSetpoints.put(CarriageLevel.Zero, 0);
    hatchSetpoints.put(CarriageLevel.HatchPickup, Constants.HATCH_PICKUP);
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
    BadLog.createTopicStr("Carriage/Is beam break?", "bool", () -> LogUtil.fromBool(this.isBeamBreakOpen()));
    BadLog.createTopicStr("Carriage/Has hatch?", "bool", () -> LogUtil.fromBool(this.hasHatch()));
  }

  public void outputTelemetry() {
    SmartDashboard.putBoolean("Carriage/Is cargo eject open?", this.isCargoEjectOpen());
    SmartDashboard.putBoolean("Carriage/Is beak open?", this.isBeakOpen());
    SmartDashboard.putBoolean("Carriage/Is beam break open?", this.isBeamBreakOpen());
    SmartDashboard.putBoolean("Carriage/Has hatch?", this.hasHatch());
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

  public boolean isBeamBreakOpen() {
    return !beamBreak.get();
  }

  public int getSetpoint(CarriageLevel level)
  {
    if (this.hasHatch()) {
      return hatchSetpoints.get(level);
    } else {
      return cargoSetpoints.get(level);
    }
  }

  public boolean hasHatch() {
    return !hatchLimitSwitch.get();
  }

}
