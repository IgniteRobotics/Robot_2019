/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DoNothing;
import frc.robot.commands.carriage.CloseBeak;
import frc.robot.commands.carriage.EjectCargo;
import frc.robot.commands.carriage.OpenBeak;
import frc.robot.commands.carriage.RetractCargo;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;

public class MoveThenEject extends CommandGroup {
  /**
   * Add your docs here.
   */
  public MoveThenEject(Elevator elevator, Carriage carriage, int hatchSetpoint, int cargoSetpoint, double ejectTimeout) {
    SmartDashboard.putBoolean("has hatch", !carriage.isHatchLimitSwitchOpen());
    System.out.println("---------------" + carriage.isHatchLimitSwitchOpen() + "--------------------------------------");
    if (!carriage.isHatchLimitSwitchOpen()) {
      addSequential(new MoveToSetpoint(elevator, hatchSetpoint));
      addSequential(new OpenBeak(carriage));
      addSequential(new DoNothing(ejectTimeout));
      addSequential(new MoveToSetpoint(elevator, hatchSetpoint + 2000));
      addSequential(new CloseBeak(carriage));
    } else {
      addSequential(new MoveToSetpoint(elevator, cargoSetpoint));
      addSequential(new EjectCargo(carriage));
      addSequential(new DoNothing(ejectTimeout));
      addSequential(new RetractCargo(carriage));
    }
  }

}
