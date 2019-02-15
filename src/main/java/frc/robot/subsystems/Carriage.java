/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Carriage extends Subsystem {

  private Solenoid cargoEject;
  private Solenoid beak;
  private Solenoid hatchEject; 
  private DigitalInput beamBreak;

  private boolean cargoEjectState;
  private boolean beakState;
  private boolean hatchEjectState;

  private Command defaultCommand;

  public Carriage(int pcmID, int cargoEjectSolenoid, int beakSolenoid, int hatchEjectSolenoid, int beamBreakID) {

    cargoEject = new Solenoid(pcmID, cargoEjectSolenoid);
    beak = new Solenoid(pcmID, beakSolenoid);
    hatchEject = new Solenoid(pcmID, hatchEjectSolenoid);
    beamBreak = new DigitalInput(beamBreakID);

  }
  
  public void setCommandDefault(Command command) {
    this.defaultCommand = command;
    initDefaultCommand();
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(defaultCommand);
  }

  private void pollCargoEject() {
    cargoEjectState = cargoEject.get();
  }

  private void pollBeak() {
    beakState = beak.get();
  }

  private void pollHatchEject() {
    hatchEjectState = hatchEject.get();
  }

  public boolean isCargoEjectOpen() {
    pollCargoEject();
    return cargoEjectState;
  }

  public boolean isBeakOpen() {
    pollBeak();
    return beakState;
  }

  public boolean isHatchEjectOpen() {
    pollHatchEject();
    return hatchEjectState;
  }

  public void ejectCargo() {
    cargoEject.set(true);
  }

  public void openBeak() {
    beak.set(true);
  }

  public void ejectHatch() {
    hatchEject.set(true);
  }

  public void retractEjectCargo() {
    cargoEject.set(false);
  }

  public void retractBeak() {
    beak.set(false);
  }

  public void retractEjectHatch() {
    hatchEject.set(false);
  }

  public boolean isBeakBreakOpen() {
    return beamBreak.get(); //TODO: may need negation
  }

  public void outputTelemetry() {
      
  }

}
