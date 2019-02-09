/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Elevator extends Subsystem {

  private WPI_TalonSRX elevatorMaster;
  private WPI_VictorSPX elevatorFollower;

  private Command defaultCommand;

  public Elevator(int elevatorMasterID, int elevatorFollowerID) {

    elevatorMaster = new WPI_TalonSRX(elevatorMasterID);
    elevatorFollower = new WPI_VictorSPX(elevatorFollowerID);
    
  }

  public void setDefault(Command command){
    defaultCommand = command;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
