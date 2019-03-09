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
import frc.robot.commands.driveTrain.AutoDrivePlaceItem;
import frc.robot.commands.driveTrain.DriveToDistance;
import frc.robot.commands.elevator.MoveOpenLoop;
import frc.robot.commands.elevator.MoveThenEject;
import frc.robot.commands.carriage.CloseBeak;
import frc.robot.commands.carriage.EjectCargo;
import frc.robot.commands.carriage.OpenBeak;
import frc.robot.commands.intake.IntakeCargo;
import frc.robot.commands.HatchDriveForwardCurrent;
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

	//driver
	public Button level3 = new JoystickButton(driverJoystick, BUTTON_Y);
	public Button level2 = new JoystickButton(driverJoystick, BUTTON_B);
	public Button level1 = new JoystickButton(driverJoystick, BUTTON_A);
	public Button mmDriveTest = new JoystickButton(driverJoystick, BUTTON_X);
	public Button openIntake = new JoystickButton(driverJoystick, BUTTON_LEFT_BUMPER);

    public OI (DriveTrain driveTrain, Carriage carriage, Elevator elevator, Intake intake, JetsonSink jetsonSink) {

		//manipulator		
		openBeak.toggleWhenPressed(new HatchDriveForwardCurrent(carriage, driveTrain));
		
		ejectCargo.whenPressed(new EjectCargo(carriage));
		ejectCargo.whenReleased(new RetractCargo(carriage));

        jogSwitch.whileHeld(new MoveOpenLoop(elevator, manipulatorJoystick, AXIS_LEFT_STICK_Y, Constants.ELEVATOR_JOG_DEADBAND));

		//driver
		openIntake.toggleWhenPressed(new IntakeCargo(intake, carriage));
		level3.whenPressed(new MoveThenEject(elevator, carriage, CarriageLevel.Level3, Constants.EJECT_TIMEOUT));
		level2.whenPressed(new MoveThenEject(elevator, carriage, CarriageLevel.Level2, Constants.EJECT_TIMEOUT));
		level1.whenPressed(new MoveThenEject(elevator, carriage, CarriageLevel.Level1, Constants.EJECT_TIMEOUT));
		
		mmDriveTest.whenPressed(new AutoDrivePlaceItem(elevator, driveTrain, carriage, CarriageLevel.Level2, Constants.EJECT_TIMEOUT));
    }

}
