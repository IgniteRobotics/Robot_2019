/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Optional;

import badlog.lib.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.commands.driveTrain.arcadeDrive;
import frc.robot.commands.elevator.HoldPosition;

import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Jetson;
import frc.robot.OI;
import frc.robot.util.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static Carriage carriage;
  private static DriveTrain driveTrain;
  private static Elevator elevator;
  private static Intake intake;
  private static Climber climber;

  private static arcadeDrive arcadeDrive;
  private static HoldPosition holdPosition;

  private static Jetson jetson;

  private BadLog logger;

  private SubsystemManager subsystemManager;
  private static OI oi;

  private long startTime;

  public static double turn1;
  public static double drive1;
  public static double turn2;
  public static double drive2;

  private static boolean lockVisionValues;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    startTime = System.nanoTime();

    lockVisionValues = false;

    logger = BadLog.init("/home/lvuser/log/" + LogUtil.genSessionName() + ".bag");

    initializeSubsystems();
    initializeCommands();

    writeSystemLog();

    logger.finishInitialization();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */

  @Override
  public void robotPeriodic() {
    subsystemManager.outputTelemetry();
    subsystemManager.zeroSensorsFromDashboard();
    jetson.turnOffToggle();
  }

  private void matchInit() {
    jetson.turnOnLed();
    Scheduler.getInstance().run();
    climber.closeSuction();
  }

  private void matchPeriodic() {
    double currentTime = ((double) (System.nanoTime() - startTime)) / 1_000_000_000d;
    BadLog.publish("Time", currentTime);

    Scheduler.getInstance().run();

    logger.updateTopics();
    logger.log();

    if (!lockVisionValues) {
      //updateVisionValues();
    }

  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    jetson.turnOffLed();
  }

  @Override
  public void disabledPeriodic() {
    if (elevator.isFwdLimitTripped()) {
      elevator.zeroSensors();
    }
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    subsystemManager.zeroAllSensors();
    subsystemManager.outputTelemetry();
    matchInit();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    matchPeriodic();
  }

  @Override
  public void teleopInit() {
    matchInit();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    matchPeriodic();
  }

  private void initializeCommands() {

    oi = new OI(driveTrain, carriage, elevator, intake, climber);

    arcadeDrive = new arcadeDrive(driveTrain, oi.driverJoystick, oi.AXIS_LEFT_STICK_Y, oi.AXIS_RIGHT_STICK_X,
        Constants.DRIVE_DEADBAND, jetson);
    driveTrain.establishDefaultCommand(arcadeDrive);

    holdPosition = new HoldPosition(elevator);
    elevator.establishDefaultCommand(holdPosition);

  }

  private void initializeSubsystems() {

    carriage = new Carriage(RobotMap.pcmID, RobotMap.cargoEjectSolenoid, RobotMap.beakSolenoid,
        RobotMap.carriageBeamBreakID);
    driveTrain = new DriveTrain(RobotMap.leftMasterID, RobotMap.leftFollowerID, RobotMap.rightMasterID,
        RobotMap.rightFollowerID);
    elevator = new Elevator(RobotMap.elevatorMasterID, RobotMap.elevatorFollowerID);
    intake = new Intake(RobotMap.pcmID, RobotMap.intakeMotorID, RobotMap.intakeSolenoidOpen,
        RobotMap.intakeSolenoidClose, RobotMap.intakeBeamBreakID);
    climber = new Climber(RobotMap.climberMotorID, RobotMap.suctionIDForward, RobotMap.suctionIDReverse);
    jetson = new Jetson(RobotMap.jetsonPowerDioID, RobotMap.relayID);

    subsystemManager = new SubsystemManager();

    subsystemManager.addSubsystems(carriage, driveTrain, elevator, intake, jetson, climber);

    subsystemManager.zeroAllSensors();
    subsystemManager.zeroSensorsFromDashboard();
    subsystemManager.outputTelemetry();

  }

  private void writeSystemLog() {
    BadLog.createValue("Start Time", LogUtil.getTimestamp());
    BadLog.createValue("Event Name", Optional.ofNullable(DriverStation.getInstance().getEventName()).orElse(""));
    BadLog.createValue("Match Type", DriverStation.getInstance().getMatchType().toString());
    BadLog.createValue("Match Number", "" + DriverStation.getInstance().getMatchNumber());
    BadLog.createValue("Alliance", DriverStation.getInstance().getAlliance().toString());
    BadLog.createValue("Location", "" + DriverStation.getInstance().getLocation());

    BadLog.createTopicSubscriber("Time", "s", DataInferMode.DEFAULT, "hide", "delta", "xaxis");

    BadLog.createTopicStr("System/Browned Out", "bool", () -> LogUtil.fromBool(RobotController.isBrownedOut()));
    BadLog.createTopic("System/Battery Voltage", "V", () -> RobotController.getBatteryVoltage());
    BadLog.createTopicStr("System/FPGA Active", "bool", () -> LogUtil.fromBool(RobotController.isSysActive()));
    BadLog.createTopic("Match Time", "s", () -> DriverStation.getInstance().getMatchTime());
  }

  private void updateVisionValues() {
    turn1 = jetson.getTurn1();
    drive1 = jetson.getDistance1();
    turn2 = jetson.getTurn2();
    drive2 = jetson.getDistance2();
  }

  public static void lockVision() {
    lockVisionValues = true;
  }

  public static void unlockVision() {
    lockVisionValues = false;
  }

}
