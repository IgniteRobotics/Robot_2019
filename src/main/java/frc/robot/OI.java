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
import frc.robot.commands.carriage.OpenBeak;
import frc.robot.commands.carriage.RetractCargo;
import frc.robot.commands.elevator.MoveOpenLoop;
import frc.robot.commands.elevator.MoveThenEject;
import frc.robot.commands.carriage.CloseBeak;
import frc.robot.commands.carriage.EjectCargo;
import frc.robot.commands.intake.CloseRollIntake;
import frc.robot.commands.intake.OpenIntake;
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
    
    public Button openBeak = new JoystickButton(manipulatorJoystick, BUTTON_A);
	public Button openIntake = new JoystickButton(manipulatorJoystick, BUTTON_B);
	public Button ejectCargo = new JoystickButton(manipulatorJoystick, BUTTON_X);
    public Button jogSwitch = new JoystickButton(manipulatorJoystick, BUTTON_Y);

	public Button level3 = new JoystickButton(manipulatorJoystick, BUTTON_LEFT_BUMPER);
	public Button level2 = new JoystickButton(manipulatorJoystick, BUTTON_RIGHT_BUMPER);
	public Button level1 = new JoystickButton(manipulatorJoystick, BUTTON_BACK);

    public OI (DriveTrain driveTrain, Carriage carriage, Elevator elevator, Intake intake) {

        openIntake.whileHeld(new OpenIntake(intake, carriage));
        openIntake.whenReleased((new CloseRollIntake(intake, carriage)));

        openBeak.whileHeld(new OpenBeak(carriage));
        openBeak.whenReleased(new CloseBeak(carriage));
		
		ejectCargo.whenPressed(new EjectCargo(carriage));
		ejectCargo.whenReleased(new RetractCargo(carriage));

		level3.whenPressed(new MoveThenEject(elevator, carriage, Constants.ROCKET_HATCH_L3, Constants.ROCKET_CARGO_L3, 0.5));
		level2.whenPressed(new MoveThenEject(elevator, carriage, Constants.ROCKET_HATCH_L2, Constants.ROCKET_CARGO_L2, 0.5));
		level1.whenPressed(new MoveThenEject(elevator, carriage, Constants.ROCKET_HATCH_L1, Constants.ROCKET_CARGO_L1, 0.5));

        jogSwitch.whileHeld(new MoveOpenLoop(elevator, manipulatorJoystick, AXIS_LEFT_STICK_Y, Constants.ELEVATOR_JOG_DEADBAND));

    }

}
