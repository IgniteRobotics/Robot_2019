/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  private WPI_TalonSRX leftmaster = new WPI_TalonSRX(1);
  private WPI_VictorSPX leftfollower = new WPI_VictorSPX(2);
  private WPI_TalonSRX rightmaster = new WPI_TalonSRX(4);
  private WPI_VictorSPX rightfollower = new WPI_VictorSPX(5);
  Encoder leftencoder = new Encoder(1, 2, true);
  Encoder rightencoder = new Encoder(4, 5, false);

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
