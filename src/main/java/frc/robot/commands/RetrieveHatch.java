/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.carriage.CloseBeak;
import frc.robot.commands.carriage.OpenBeak;
import frc.robot.commands.driveTrain.DriveToDistanceTimed;
import frc.robot.commands.elevator.ElevatorState;
import frc.robot.commands.elevator.MoveToSetpoint;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

public class RetrieveHatch extends CommandGroup {

  public RetrieveHatch(Elevator elevator, Carriage carriage, DriveTrain driveTrain) {
  
    setInterruptible(true);

    addSequential(new OpenBeak(carriage));
    addSequential(new CloseBeak(carriage));
    addSequential(new MoveToSetpoint(elevator, ElevatorState.HatchPickup, carriage));
    addSequential(new DriveToDistanceTimed(driveTrain, 0.5, -0.3));
    addSequential(new MoveToSetpoint(elevator, ElevatorState.Zero, carriage));
    


  }
}
