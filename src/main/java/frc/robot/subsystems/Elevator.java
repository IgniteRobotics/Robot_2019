/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  DigitalInput elevatorlimitswitch = new DigitalInput(9);
  private Solenoid solenoid1 = new Solenoid(1, 7);
  private Solenoid solenoid2 = new Solenoid(2, 7);
  private WPI_TalonSRX upmotor = new WPI_TalonSRX(8);
  private WPI_VictorSPX downmotor = new WPI_VictorSPX(9);
  Encoder elevatorencoder = new Encoder(8, 9, false);
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
