package frc.robot.subsystems.elbow_subsystem;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hang.HangingSubsystem.LatchServoPosition;

public class ElbowSubsystem extends SubsystemBase{
    public final static double START_POS_ELEVATION = 0;
    public final static double START_POS_ROTATION = 0;

    public final static double HORIZONTAL_POS_ELEVATION = -90;
    public final static double HORIZONTAL_POS_ROTATION = 90;
    public final static double CORAL_COMPENSATION = 45;

    public final static double[] INTAKE_POS = {-98.0,89.0};
    public final static double[] LOW_POS    = {-26.0,90.0};
    public final static double[] MIDS_POS   = {-51.5,0.0};
    public final static double[] HIGH_POS   = {-44,0.0};

    public final static double INTAKE_POS_ELEVATION = -98;
    public final static double INTAKE_POS_ROTATION = 89;// to be 90

    private Timer timer = new Timer();

    public static boolean hasBeenInitialized = false;

    protected static SparkMax leftElbowMotor = new SparkMax(2, MotorType.kBrushless);
    protected static SparkMax rightElbowMotor = new SparkMax(3, MotorType.kBrushless);

    public SparkMaxConfig motorConfig = new SparkMaxConfig();
    public SparkClosedLoopController leftElbowClosedLoopController = leftElbowMotor.getClosedLoopController();
    public static  RelativeEncoder leftElbowEncoder = leftElbowMotor.getEncoder();
    
    public static RelativeEncoder rightElbowEncoder = rightElbowMotor.getEncoder();
    public SparkClosedLoopController rightElbowClosedLoopController = rightElbowMotor.getClosedLoopController();

    public double ELBOW_MOTORS_GEAR_RATIO = 360/48.0 ;

    public double elevation = 0;
    public double rotation = 0;

    public double leftMotorPos = 0;
    public double rightMotorPos = 0;

    

    public ElbowSubsystem() {
        timer.start();

        motorConfig.encoder.positionConversionFactor(ELBOW_MOTORS_GEAR_RATIO)
        .velocityConversionFactor(1);
        motorConfig.smartCurrentLimit(3,4,50);

        motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1).i(0.00000).d(0.0000)
        .outputRange(-.4, .6, ClosedLoopSlot.kSlot0);
        
        
        motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1, ClosedLoopSlot.kSlot1).d(0.0005, ClosedLoopSlot.kSlot1)
        .outputRange(-.4, .4, ClosedLoopSlot.kSlot1);

        motorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.
            .maxVelocity(3000)
            .maxAcceleration(8000)
            .allowedClosedLoopError(1).positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
            

        leftElbowMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightElbowMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        leftElbowClosedLoopController.setReference(START_POS_ELEVATION, SparkMax.ControlType.kPosition); 
        rightElbowClosedLoopController.setReference(START_POS_ELEVATION, SparkMax.ControlType.kPosition); 
    }

    public static void initialize(){
        if(!hasBeenInitialized) {
            resetEncoder();

            leftElbowEncoder.setPosition(START_POS_ELEVATION);
            rightElbowEncoder.setPosition(START_POS_ELEVATION);
            hasBeenInitialized = true;
        }

        leftElbowMotor.getClosedLoopController().setReference(START_POS_ELEVATION, SparkMax.ControlType.kPosition); 
        rightElbowMotor.getClosedLoopController().setReference(-START_POS_ELEVATION, SparkMax.ControlType.kPosition); 
    }

    @Override
    public void periodic() {
      // Put code here to be run every loop
      if(timer.hasElapsed(2.0)) {
        // System.out.println("LeftElbowPos" + leftElbowEncoder.getPosition());
        // System.out.println("RightElbowPos" + rightElbowEncoder.getPosition());

        System.out.println("-----------------------");
        System.out.println("Elbow Elevation: " + getElevationPos());
        System.out.println("Elbow Rotation: " + getRotationPos());
        timer.reset();
      }
    }

    public void setElevationRotationPos(double elevation, double rotation) {
        leftMotorPos = -elevation + rotation;
        rightMotorPos = elevation + rotation;

        leftElbowClosedLoopController.setReference(leftMotorPos, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        rightElbowClosedLoopController.setReference(rightMotorPos, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    public void setElevationRotationPos(double elevation, double rotation, boolean slowMode) {
        leftMotorPos = -elevation + rotation;
        rightMotorPos = elevation + rotation;

        if (slowMode) {
            leftElbowClosedLoopController.setReference(leftMotorPos, ControlType.kPosition, ClosedLoopSlot.kSlot1);
            rightElbowClosedLoopController.setReference(rightMotorPos, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        } else {
            setElevationRotationPos(elevation, rotation);
        }
    }

    public double getElevationPos() {
        double leftMotorPos = leftElbowMotor.getEncoder().getPosition();
        double rightMotorPos = rightElbowMotor.getEncoder().getPosition();

        double elevation = (rightMotorPos - leftMotorPos) / 2.0;

        return elevation;
    }

    public double getRotationPos() {
        double leftMotorPos = leftElbowMotor.getEncoder().getPosition();
        double rightMotorPos = rightElbowMotor.getEncoder().getPosition();

        double rotation = (rightMotorPos + leftMotorPos) / 2.0;

        return rotation;
    }

    private static void resetEncoder() {
        leftElbowMotor.getEncoder().setPosition(0);
        rightElbowMotor.getEncoder().setPosition(0);
    }

    public Command resetEncoderCommand() {
        return Commands.runOnce(() -> {
            resetEncoder();
        });    
    }
}
