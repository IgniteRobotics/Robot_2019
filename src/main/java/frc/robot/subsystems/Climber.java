package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import badlog.lib.BadLog;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.LogUtil;
import frc.robot.util.Util;

public class Climber extends IgniteSubsystem {

    private WPI_TalonSRX climberMotor;

    private DoubleSolenoid suction;

    private boolean suctionState;

    private Command defaultCommand;

    private final double kP = 0;
    private final double kI = 0;
    private final double kD = 0;
    private final double kF = 0;

    private final int CRUISE_VELOCITY = 0;
    private final int MAX_ACCELERATION = 0;

    private final int TOLERANCE = 100;

    public Climber(int climberMotorID, int suctionIDForward, int suctionIDReverse) {

        climberMotor = new WPI_TalonSRX(climberMotorID);
        suction = new DoubleSolenoid(suctionIDForward, suctionIDReverse);

        climberMotor.setNeutralMode(NeutralMode.Brake);

        climberMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        climberMotor.setSensorPhase(true);
        climberMotor.setInverted(true);

        climberMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 20);
        climberMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10, 20);

        climberMotor.selectProfileSlot(3, 0);
        climberMotor.config_kF(3, kF, 10);
        climberMotor.config_kP(3, kP, 10);
        climberMotor.config_kI(3, kI, 10);
        climberMotor.config_kD(3, kD, 10);

        climberMotor.configMotionCruiseVelocity(CRUISE_VELOCITY);
        climberMotor.configMotionAcceleration(MAX_ACCELERATION);

        writeToLog();

    }

    public void establishDefaultCommand(Command command) {
        this.defaultCommand = command;
        initDefaultCommand();
    }

    public boolean checkSystem() {
        return true;
    }

    public void writeToLog() {
        BadLog.createTopic("Climber/Master Percent Output", BadLog.UNITLESS, () -> this.getPercentOutput(), "hide",
                "join:Climber/Output percents");
        BadLog.createTopic("Climber/Master Voltage", "V", () -> this.getMasterVoltage(), "hide",
                "join:Climber/Output Voltages");
        BadLog.createTopic("Climber/Master Current", "A", () -> this.getMasterCurrent(), "hide",
                "join:Climber/Output Current");
        BadLog.createTopic("Climber/Position", "ticks", () -> (double) this.getEncoderPos(), "hide",
                "join:Climber/Position");
        BadLog.createTopic("Climber/Velocity", "ticks", () -> (double) this.getEncoderVel(), "hide",
                "join:Climber/Velocity");
        BadLog.createTopicStr("Climber/Fwd limit", "bool", () -> LogUtil.fromBool(this.isFwdLimitTripped()));
        BadLog.createTopicStr("Climber/Rev limit", "bool", () -> LogUtil.fromBool(this.isRevLimitTripped()));
        BadLog.createTopicStr("Climber/Is suction open?", "bool", () -> LogUtil.fromBool(this.isSuctionOpen()));
    }

    public void outputTelemetry() {
        SmartDashboard.putNumber("Climber/Pos", this.getEncoderPos());
        SmartDashboard.putNumber("Climber/Vel", this.getEncoderVel());
        SmartDashboard.putNumber("Climber/Master voltage", this.getMasterVoltage());
        SmartDashboard.putNumber("Climber/Master current", this.getMasterCurrent());
        SmartDashboard.putNumber("Climber/Percent out", this.getPercentOutput());
        SmartDashboard.putBoolean("Climber/Is fwd limit switch tripped?", this.isFwdLimitTripped());
        SmartDashboard.putBoolean("Climber/Is rev limit switch tripped?", this.isRevLimitTripped());
        SmartDashboard.putBoolean("Climber/Is suction open?", this.isSuctionOpen());
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(this.defaultCommand);
    }

    public void setOpenLoop(double percentage) {
        climberMotor.set(ControlMode.PercentOutput, percentage);
    }

    public void setOpenLoop(double percentage, double deadband) {
        percentage = Util.applyDeadband(percentage, Constants.ELEVATOR_JOG_DEADBAND);
        setOpenLoop(percentage);
    }

    public void setMotionMagicPosition(double position) {
        climberMotor.set(ControlMode.MotionMagic, position);
    }

    public boolean isMotionMagicDone() {
        return Math.abs(climberMotor.getClosedLoopTarget() - this.getEncoderPos()) <= TOLERANCE;
    }

    public int getEncoderPos() {
        return climberMotor.getSelectedSensorPosition();
    }

    public double getEncoderVel() {
        return climberMotor.getSelectedSensorVelocity();
    }

    public double getMasterVoltage() {
        return climberMotor.getMotorOutputVoltage();
    }

    public double getPercentOutput() {
        return climberMotor.getMotorOutputPercent();
    }

    public double getMasterCurrent() {
        return climberMotor.getOutputCurrent();
    }

    public void zeroSensors() {
        climberMotor.setSelectedSensorPosition(0);
    }

    public boolean isFwdLimitTripped() {
        return climberMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public boolean isRevLimitTripped() {
        return climberMotor.getSensorCollection().isRevLimitSwitchClosed();
    }

    public void stop() {
        climberMotor.stopMotor();
    }

    public void closeSuction() {
        suction.set(DoubleSolenoid.Value.kForward);
        suctionState = false;
    }

    public void openSuction() {
        suction.set(DoubleSolenoid.Value.kReverse);
        suctionState = true;
    }

    public void toggleSuction() {
        if (suctionState) {
            closeSuction();
        } else {
            openSuction();
        }
    }

    public boolean isSuctionOpen() {
        return this.suctionState;
    }

}