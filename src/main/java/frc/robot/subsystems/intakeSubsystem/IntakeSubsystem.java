package frc.robot.subsystems.intakeSubsystem;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;



public class IntakeSubsystem extends SubsystemBase {    
    private Timer timer = new Timer();
    private boolean enableTelemetry;

    protected SparkMax intakeMotor = new SparkMax(4, MotorType.kBrushless);

    protected SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    protected SparkClosedLoopController intakeMotorController = intakeMotor.getClosedLoopController();
    public RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    
    // private static final double INTAKE_VOLTAGE = 5.0;
    // private static final double OUTAKE_VOLTAGE = 5.0;

    private static final double CORAL_HOLDING_VELOCITY =  0.0;
    private static final double CORAL_INTAKE_VELOCITY  =  12.0; // In terms of coral
    private static final double CORAL_OUTAKE_VELOCITY  =  2.0;

    private static final double ALGEA_HOLDING_VELOCITY = 4.0;
    private static final double ALGEA_OUTAKE_VELOCITY =  2.0; // In terms of algea
    private static final double ALGEA_INTAKE_VELOCITY =  12.0;
    
    public IntakeSubsystem(boolean enableTelemetry) {
        this.enableTelemetry = enableTelemetry;
        // Initialize the subsystem here
        timer.start();
        intakeMotorConfig.smartCurrentLimit(1,8,50);

        intakeMotorConfig.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.
            .maxVelocity(3000)
            .maxAcceleration(8000)
            .allowedClosedLoopError(1).positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        intakeMotorController.setReference(0, SparkMax.ControlType.kVoltage);    
    }

    

    @Override
    public void periodic() {
        // Put code here to be run every loop
        if(timer.hasElapsed(2.0)) {
            if (enableTelemetry){
                System.out.println("intake output velocity" + intakeEncoder.getVelocity());
                System.out.println("algaeMode is: " + RobotContainer.AlgaeMode);
            }

        timer.reset();
        }
    }

    public void setTargetVoltage(double targetVelocity){
        intakeMotorController.setReference(targetVelocity, ControlType.kVoltage);
    }

    public void coralIntake(){
        setTargetVoltage(CORAL_INTAKE_VELOCITY);
    }

    public void coralOuttake(){
        setTargetVoltage(-CORAL_OUTAKE_VELOCITY);
    }

    public void coralStop(){
        setTargetVoltage(CORAL_HOLDING_VELOCITY);
    }

    public void algeaIntake(){
        setTargetVoltage(-ALGEA_INTAKE_VELOCITY);
    }

    public void algeaOuttake(){
        setTargetVoltage(ALGEA_OUTAKE_VELOCITY);
    }

    public void algeaStop(){
        setTargetVoltage(-ALGEA_HOLDING_VELOCITY);
    }

    public Command coralOuttakeCommand() {
        return this.runOnce(() -> {
            coralOuttake();
        });
    }
}