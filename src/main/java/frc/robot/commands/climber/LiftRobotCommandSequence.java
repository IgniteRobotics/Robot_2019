/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.vision.*;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;

public class LiftRobotCommandSequence extends CommandGroup {

  public LiftRobotCommandSequence(Climber climber, Joystick manipulatorJoystick, int axisId, int acceptButtonId, int deadband) {

    addSequential(new RaiseLiftCarriage(climber));
    addSequential(new PlaceFoot(climber));
    addSequential(new AcceptVacuum(climber, manipulatorJoystick, axisId, acceptButtonId, deadband));
    addSequential(new LiftRobot(climber));

  }

}