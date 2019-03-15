package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.CarriageLevel;
import frc.robot.commands.DoNothing;
import frc.robot.commands.driveTrain.DriveToDistance;
import frc.robot.commands.carriage.CarriageClose;
import frc.robot.commands.carriage.CarriageOpen;
import frc.robot.commands.elevator.MoveToSetpoint;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class AutoDrivePlaceItem extends CommandGroup {

    public AutoDrivePlaceItem(Elevator elevator, Intake intake, DriveTrain driveTrain, Carriage carriage, CarriageLevel level, double ejectTimeout) {

    addSequential(new DriveToDistance(driveTrain));
    addSequential(new MoveToSetpoint(elevator, level, carriage, intake));
    addSequential(new CarriageOpen(carriage, intake));
    addSequential(new DoNothing(ejectTimeout));
    addSequential(new CarriageClose(carriage));
    addSequential(new MoveToSetpoint(elevator, CarriageLevel.Zero, carriage, intake));
    }

}