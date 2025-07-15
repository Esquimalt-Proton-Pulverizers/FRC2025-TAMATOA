package frc.robot.subsystems.intake_subsystem;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class IntakeSubsystem extends SubsystemBase {
    public boolean isCompetitionRobot;
    
    private Timer timer = new Timer();

    protected SparkMax intakeMotor = new SparkMax(4, MotorType.kBrushless);
    //add new victor spx
    //private WPI_VictorSPX victor = new WPI_VictorSPX(0);
    protected SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    protected SparkClosedLoopController intakeMotorController = intakeMotor.getClosedLoopController();
    public RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    
    private static final double INTAKE_VOLTAGE = 5;
    private static final double OUTAKE_VOLTAGE = 5;

    private static final double HOLDING_VELOCITY = 0;
    private static final double INTAKE_VELOCITY = 10000;//in terms of coral
    private static final double OUTAKE_VELOCITY = -10000;
    
    public IntakeSubsystem() {
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
        // if(timer.hasElapsed(2.0)) {
        // System.out.println("intake output velocity" + intakeEncoder.getVelocity());
        // timer.reset();
        // }
    }
    public void setTargetVoltage(double targetVelocity){
        intakeMotorController.setReference(targetVelocity, ControlType.kVoltage);
    }

    public void intake(){
        setTargetVoltage(INTAKE_VELOCITY);
    }

    public void outtake(){
        setTargetVoltage(OUTAKE_VELOCITY);
    }

    public void stop(){
        setTargetVoltage(HOLDING_VELOCITY);
    }
}