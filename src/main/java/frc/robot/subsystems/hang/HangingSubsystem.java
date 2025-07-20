package frc.robot.subsystems.hang;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class HangingSubsystem extends SubsystemBase {
    // Constants 
    private final int WINCH_MOTOR_CAN_ID = 10;
    private final int LATCH_SERVO_PORT = 0;
    private final int INTAKE_MOTOR_CANID = 11;

    private final double INTAKE_VOLTAGE = 6.0;
    private final double HOLDING_VOLTAGE = 0.0;

    // Hardware
    private  SparkMax winchMotor;
    private SparkClosedLoopController winchController;
    private RelativeEncoder winchEncoder; 
    private Servo latchServo;
    private SparkMax intakeMotor;
    private SparkClosedLoopController intakeController;

    
    public HangingSubsystem() {
        winchMotor = new SparkMax(WINCH_MOTOR_CAN_ID, MotorType.kBrushless);
        
        SparkMaxConfig winchConfig = new SparkMaxConfig();
        winchConfig
            .inverted(true)
            .smartCurrentLimit(1, 8, 50);

        winchConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        winchConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.1, 0.0, 0.0)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot0);

        winchMotor.configure(
            winchConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kNoPersistParameters
        );

        winchEncoder = winchMotor.getEncoder();
        winchController = winchMotor.getClosedLoopController();

        latchServo = new Servo(LATCH_SERVO_PORT);
        setLatchServoPosition(LatchServoPosition.FREE);

        SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
        
        intakeMotor = new SparkMax(INTAKE_MOTOR_CANID, MotorType.kBrushless);
        intakeMotorConfig.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.
            .maxVelocity(3000)
            .maxAcceleration(8000)
            .allowedClosedLoopError(1).positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
            

        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        intakeController = intakeMotor.getClosedLoopController();
        intakeController.setReference(0, SparkMax.ControlType.kVoltage);
        intakeMotor.getEncoder();    

        new Timer();
    }

    private void setLatchServoPosition(LatchServoPosition position) {
        latchServo.set(position.value);
    }

    public void setWinchPosition(double position) {
        winchController.setReference(position, ControlType.kPosition);
    }

    public void setWinchPosition(WinchPosition position) {
        setWinchPosition(position.value);
    }

    public double getWinchPosition() {
        return winchEncoder.getPosition();
    }

    // Commands
    public SequentialCommandGroup extend() {
        return new SequentialCommandGroup(
            release(),
            new WinchToPositionCommand(this, -3),
            new WinchToPositionCommand(this, WinchPosition.EXTENDED)
        );
    }

    public SequentialCommandGroup retract() {
        return new SequentialCommandGroup(
            latch(),
            new WinchToPositionCommand(this, WinchPosition.RETRACTED)  
        );
    }

    public Command manualRetract() {
        return Commands.runOnce(() -> {
            winchController.setReference(-5, ControlType.kVoltage);
            setLatchServoPosition(LatchServoPosition.LATCHED);
        });    
    }

    public Command resetWinch() {
        return Commands.runOnce(() -> {
            winchController.setReference(0, ControlType.kVoltage);
            winchMotor.getEncoder().setPosition(0);
        });
    }

    public Command release() {
        return Commands.runOnce(() -> setLatchServoPosition(LatchServoPosition.FREE));    
    }

    public Command latch() {
        return Commands.runOnce(() -> setLatchServoPosition(LatchServoPosition.LATCHED));    
    }

    @Override public void periodic() {
        SmartDashboard.putNumber("Servo Position", latchServo.getPosition());
    }

    public enum LatchServoPosition {
        LATCHED(1),
        FREE(0.5);

        double value;

        private LatchServoPosition(double value) {
            this.value = value;
        }
    }

    public enum WinchPosition {
        RETRACTED(0),
        EXTENDED(690);

        double value;

        private WinchPosition(double value) {
            this.value = value;
        }
    }

    public Command intake(){
        return Commands.runOnce(() -> intakeController.setReference(INTAKE_VOLTAGE, ControlType.kVoltage));    
    }

    public Command stop(){
        return Commands.runOnce(() -> intakeController.setReference(HOLDING_VOLTAGE, ControlType.kVoltage));    
    }
}