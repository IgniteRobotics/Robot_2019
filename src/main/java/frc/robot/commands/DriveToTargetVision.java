/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.vision.*;
import frc.robot.subsystems.DriveTrain;

public class DriveToTargetVision extends CommandGroup {

  public DriveToTargetVision(DriveTrain driveTrain) {

    addSequential(new TurnToAngleVision(driveTrain, true, VisionData.TURN_1));
    addSequential(new DriveToDistanceVision(driveTrain, true, VisionData.DISTANCE_1));
    addSequential(new TurnToAngleVision(driveTrain, true, VisionData.TURN_2));
    addSequential(new DriveToDistanceVision(driveTrain, false, VisionData.DISTANCE_2));

  }

}
