/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.carriage.RetractCargo;
import frc.robot.commands.carriage.ToggleBeak;
import frc.robot.commands.driveTrain.TurnToAngle;
import frc.robot.commands.RetrieveHatch;
import frc.robot.commands.elevator.EjectThenHome;
import frc.robot.commands.elevator.ElevatorState;
import frc.robot.commands.elevator.MoveOpenLoop;
import frc.robot.commands.elevator.MoveToSetpoint;
import frc.robot.commands.carriage.EjectCargo;
import frc.robot.commands.intake.CloseIntake;
import frc.robot.commands.intake.IntakeCargo;
import frc.robot.commands.intake.OuttakeCargo;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  
	private final int DRIVER_JOYSTICK = 0;
	private final int MANIPULATOR_JOYSTICK = 1;

	public final int BUTTON_A = 1;
	public final int BUTTON_B = 2;
	public final int BUTTON_X = 3;
	public final int BUTTON_Y = 4;
	public final int BUTTON_LEFT_BUMPER = 5;
	public final int BUTTON_RIGHT_BUMPER = 6;
	public final int BUTTON_BACK = 7;
	public final int BUTTON_START = 8;
	public final int BUTTON_LEFT_STICK = 9;
	public final int BUTTON_RIGHT_STICK = 10;

	public final int AXIS_LEFT_STICK_X = 0;
	public final int AXIS_LEFT_STICK_Y = 1;
	public final int AXIS_LEFT_TRIGGER = 2;
	public final int AXIS_RIGHT_TRIGGER = 3;
	public final int AXIS_RIGHT_STICK_X = 4;
    public final int AXIS_RIGHT_STICK_Y = 5;
    
	public Joystick driverJoystick = new Joystick(DRIVER_JOYSTICK);
    public Joystick manipulatorJoystick = new Joystick(MANIPULATOR_JOYSTICK);

	//manipulator
    public Button openBeak = new JoystickButton(manipulatorJoystick, BUTTON_A);
	public Button ejectCargo = new JoystickButton(manipulatorJoystick, BUTTON_X);
	public Button jogSwitch = new JoystickButton(manipulatorJoystick, BUTTON_Y);
	public Button retrieveHatch = new JoystickButton(manipulatorJoystick, BUTTON_B);
	public Button outtakeCargo = new JoystickButton(manipulatorJoystick, BUTTON_RIGHT_BUMPER);
	public Button intakeCargo = new JoystickButton(manipulatorJoystick, BUTTON_LEFT_BUMPER);

	//driver
	public Button level3 = new JoystickButton(driverJoystick, BUTTON_Y);
	public Button level2 = new JoystickButton(driverJoystick, BUTTON_B);
	public Button level1 = new JoystickButton(driverJoystick, BUTTON_A);
	public Button cargoShipCargo = new JoystickButton(driverJoystick, BUTTON_X);
	public Button ejectThenHome = new JoystickButton(driverJoystick, BUTTON_RIGHT_BUMPER);

    public OI (DriveTrain driveTrain, Carriage carriage, Elevator elevator, Intake intake) {
		
		//manipulator				
		openBeak.whenPressed(new ToggleBeak(carriage));
		ejectCargo.whileHeld(new EjectCargo(carriage));
		ejectCargo.whenReleased(new RetractCargo(carriage));

        jogSwitch.whileHeld(new MoveOpenLoop(elevator, manipulatorJoystick, AXIS_LEFT_STICK_Y, Constants.ELEVATOR_JOG_DEADBAND));

		outtakeCargo.whileHeld(new OuttakeCargo(intake, carriage));
		outtakeCargo.whenReleased(new CloseIntake(intake, carriage));
		intakeCargo.toggleWhenPressed(new IntakeCargo(intake, carriage));

		retrieveHatch.whenPressed(new RetrieveHatch(elevator, carriage, driveTrain));

		//driver
		level3.whenPressed(new MoveToSetpoint(elevator, ElevatorState.Level3, carriage));
		level2.whenPressed(new MoveToSetpoint(elevator, ElevatorState.Level2, carriage));
		level1.whenPressed(new MoveToSetpoint(elevator, ElevatorState.Level1, carriage));
		cargoShipCargo.whenPressed(new MoveToSetpoint(elevator, ElevatorState.CargoShipCargo, carriage));
		
		ejectThenHome.whenPressed(new EjectThenHome(elevator, carriage, Constants.EJECT_TIMEOUT, driveTrain));
    }

}
