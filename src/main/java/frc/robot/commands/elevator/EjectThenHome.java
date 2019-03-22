/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DoNothing;
import frc.robot.commands.carriage.CarriageClose;
import frc.robot.commands.carriage.CarriageOpen;
import frc.robot.commands.driveTrain.DriveToDistanceTimedConditional;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

public class EjectThenHome extends CommandGroup {

  public EjectThenHome(Elevator elevator, Carriage carriage, double ejectTimeout, DriveTrain driveTrain) {

    setInterruptible(true);

    addSequential(new CarriageOpen(carriage));
    addSequential(new DoNothing(ejectTimeout));
    addSequential(new DriveToDistanceTimedConditional(driveTrain, carriage, Constants.EJECT_THEN_HOME_DRIVE_TIME, Constants.EJECT_THEN_HOME_DRIVE_POWER));
    addSequential(new CarriageClose(carriage));
    addSequential(new MoveToSetpoint(elevator, ElevatorState.Zero, carriage));


  }
  
}
