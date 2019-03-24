/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Intake;

public class CloseRollIntake extends CommandGroup {

  public CloseRollIntake(Intake intake, Carriage carriage) {

    addParallel(new CloseIntake(intake, carriage));
    addSequential(new RollInCargoUntilBeam(carriage, intake, Constants.INTAKE_POWER));
  
  }
}
