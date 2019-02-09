/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.arcadeDrive;
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
  private Command defaultCommand;
  private DifferentialDrive drive = new DifferentialDrive(leftmaster, rightmaster);
  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public DriveTrain () {
    leftfollower.follow(leftmaster);
    rightfollower.follow(rightmaster);   
  }
  public void arcadeDrive(double power, double rotation){
    drive.arcadeDrive(power, rotation);
  }
  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(this.defaultCommand);
}
public void setDefault(arcadeDrive arcadeDrive){
    defaultCommand = arcadeDrive;
}
}
