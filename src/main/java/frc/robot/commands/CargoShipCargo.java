/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.CarriageLevel;
import frc.robot.commands.carriage.EjectCargo;
import frc.robot.commands.carriage.RetractCargo;
import frc.robot.commands.driveTrain.DriveToDistanceTimed;
import frc.robot.commands.elevator.MoveToSetpoint;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

public class CargoShipCargo extends CommandGroup {

  public CargoShipCargo(Elevator elevator, Carriage carriage, DriveTrain driveTrain) {
    
    addSequential(new DriveToDistanceTimed(driveTrain, carriage, 0.25, -0.3, true, false));
    addSequential(new MoveToSetpoint(elevator, CarriageLevel.CargoShipCargo, carriage));
    addSequential(new EjectCargo(carriage));
    addSequential(new DoNothing(0.5));
    addSequential(new RetractCargo(carriage));
    addSequential(new MoveToSetpoint(elevator, CarriageLevel.Zero, carriage));
    
  }
}
