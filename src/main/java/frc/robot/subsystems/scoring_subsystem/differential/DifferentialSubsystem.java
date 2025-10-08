package frc.robot.subsystems.scoring_subsystem.differential;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DifferentialSubsystem extends SubsystemBase{
    public final static double START_POS_ELEVATION = 0.0;
    public final static double START_POS_ROTATION  = 0.0;

    public final static double HORIZONTAL_POS_ELEVATION = -90.0;
    public final static double HORIZONTAL_POS_ROTATION  =  90.0;
    public final static double CORAL_COMPENSATION       =  45.0;

    public final static double[] HOMING_POS = {START_POS_ELEVATION - 5.0, START_POS_ROTATION};
    public final static double[] INTAKE_POS = {-98.0, 90.0};
    public final static double[] LOW_POS    = {-26.0, 90.0};
    public final static double[] MIDS_POS   = {-51.5,  0.0};
    public final static double[] HIGH_POS   = {-51.5,  0.0};
    public final static double[] CORAL_POS  = {-26.0, 90.0};

    public static final double MIN_ELEVATION = -105.0;
    public static final double MAX_ELEVATION =    0.0;
    public static final double MIN_ROTATION =  -180.0;
    public static final double MAX_ROTATION =   180.0;

    public static  double kA = 0.0007;
    public static  double kV = 0.0008;
    public static  double kG = 0.6;
    public static  double MAX_ACCELERATION = 350;
    public static  double MAX_VELOCITY = 850;
    public static  double MAX_DECELERATION = 350;

    public double ELBOW_MOTORS_GEAR_RATIO = 360/48.0 ;

    private Timer timer = new Timer();
    private boolean enableTelemetry = false;

    public static boolean hasBeenInitialized = false;

    public static SparkMax leftElbowMotor  = new SparkMax(2, MotorType.kBrushless);
    public static SparkMax rightElbowMotor = new SparkMax(3, MotorType.kBrushless);

    public SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    public SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

    protected SparkClosedLoopController leftElbowClosedLoopController = leftElbowMotor.getClosedLoopController();
    public static  RelativeEncoder leftElbowEncoder = leftElbowMotor.getEncoder();
    
    public static RelativeEncoder rightElbowEncoder = rightElbowMotor.getEncoder();
    protected SparkClosedLoopController rightElbowClosedLoopController = rightElbowMotor.getClosedLoopController();

    public double elevation = 0;
    public double rotation = 0;

    public double leftMotorPos = 0;
    public double rightMotorPos = 0;

    public static double targetElevationPos;
    public static double targetRotationPos;

    public DifferentialSubsystem(boolean enableTelemetry) {
        this.enableTelemetry = enableTelemetry;
        timer.start();

        leftMotorConfig.encoder.positionConversionFactor(ELBOW_MOTORS_GEAR_RATIO)
            .velocityConversionFactor(1);
        leftMotorConfig.smartCurrentLimit(30,20,50);
        double upPIDLimit = 0.5;
        double downPIDLimit = 0.5;
        double P = 0.05;
        SmartDashboard.putNumber("Differential kV", kV);
        SmartDashboard.putNumber("Differential kA", kA);
        SmartDashboard.putNumber("Differential kG", kG);
        SmartDashboard.putNumber("Differential MaxA", MAX_ACCELERATION);
        SmartDashboard.putNumber("Differential MaxD", MAX_DECELERATION);
        SmartDashboard.putNumber("Differential MAxV", MAX_VELOCITY);

        leftMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(P).d(0.0000)
            .outputRange(-upPIDLimit, downPIDLimit, ClosedLoopSlot.kSlot0);
        
        
        leftMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(P, ClosedLoopSlot.kSlot1).d(0.0000, ClosedLoopSlot.kSlot1)
            .outputRange(-upPIDLimit/2, downPIDLimit/2, ClosedLoopSlot.kSlot1);

        rightMotorConfig.encoder.positionConversionFactor(ELBOW_MOTORS_GEAR_RATIO)
            .velocityConversionFactor(1);
        rightMotorConfig.smartCurrentLimit(30,20,50);

        rightMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(P).d(0.0000)
            .outputRange(-downPIDLimit, upPIDLimit, ClosedLoopSlot.kSlot0);
        
        
        rightMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(P, ClosedLoopSlot.kSlot1).d(0.0000, ClosedLoopSlot.kSlot1)
            .outputRange(-downPIDLimit/2, upPIDLimit/2, ClosedLoopSlot.kSlot1);
           

        leftElbowMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightElbowMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        leftElbowClosedLoopController.setReference(START_POS_ELEVATION, SparkMax.ControlType.kPosition); 
        rightElbowClosedLoopController.setReference(START_POS_ELEVATION, SparkMax.ControlType.kPosition); 
    }

    public void initialize(){
        if(!hasBeenInitialized) {
            resetEncoder();

            leftElbowEncoder.setPosition(START_POS_ELEVATION);
            rightElbowEncoder.setPosition(START_POS_ELEVATION);
            hasBeenInitialized = true;
        }

        leftElbowMotor.getClosedLoopController().setReference(START_POS_ELEVATION, SparkMax.ControlType.kPosition,ClosedLoopSlot.kSlot1); 
        rightElbowMotor.getClosedLoopController().setReference(-START_POS_ELEVATION, SparkMax.ControlType.kPosition,ClosedLoopSlot.kSlot1); 
    }

    @Override
    public void periodic() {
      // Put code here to be run every loop
      if(timer.hasElapsed(2.0)) {
        if (enableTelemetry){
            System.out.println("-----------------------");
            System.out.println("Elbow Elevation: " + getElevationPos());
            System.out.println("Elbow Rotation: " + getRotationPos());
            System.out.println("FFG: " + calculateGravityFF(getElevationPos()));
        }
        timer.reset();
        
      }
    }
    public void setElevationRotationPos(double elevation, double rotation, double feedForward) {
        targetElevationPos = elevation;
        targetRotationPos = rotation;

        leftMotorPos = -targetElevationPos + targetRotationPos;
        rightMotorPos = targetElevationPos + targetRotationPos;

        double ff = feedForward;
        leftElbowClosedLoopController.setReference(leftMotorPos, ControlType.kPosition, ClosedLoopSlot.kSlot0,-ff);
        rightElbowClosedLoopController.setReference(rightMotorPos, ControlType.kPosition, ClosedLoopSlot.kSlot0,ff);
    }
    public void setElevationRotationPos(double elevation, double rotation) {
        targetElevationPos = elevation;
        targetRotationPos = rotation;

        leftMotorPos = -targetElevationPos + targetRotationPos;
        rightMotorPos = targetElevationPos + targetRotationPos;

        double ff = calculateGravityFF(elevation);
        leftElbowClosedLoopController.setReference(leftMotorPos, ControlType.kPosition, ClosedLoopSlot.kSlot0,-ff);
        rightElbowClosedLoopController.setReference(rightMotorPos, ControlType.kPosition, ClosedLoopSlot.kSlot0,ff);
    }
    public void setElevationRotationPos(double elevation, double rotation, boolean slowMode) {
        targetElevationPos = elevation;
        targetRotationPos = rotation;

        leftMotorPos = -targetElevationPos + targetRotationPos;
        rightMotorPos = targetElevationPos + targetRotationPos;

        double ff = calculateGravityFF(elevation);
        if (slowMode) {
            leftElbowClosedLoopController.setReference(leftMotorPos, ControlType.kPosition, ClosedLoopSlot.kSlot1, -ff);
            rightElbowClosedLoopController.setReference(rightMotorPos, ControlType.kPosition, ClosedLoopSlot.kSlot1, ff);
        } else {
            setElevationRotationPos(elevation, rotation);
        }
    }
    public double calculateGravityFF(double elevation){
        return Math.sin(Math.toRadians(-elevation)) * kG;//0.3 would be the power required to hold the arm at 90 degrees horizontally
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

    public static void resetEncoder() {
        leftElbowMotor.getEncoder().setPosition(0);
        rightElbowMotor.getEncoder().setPosition(0);
    }

    public Command resetEncoderCommand() {
        return Commands.runOnce(() -> {
            resetEncoder();
        });    
    }

    public double getTargetElevationPosition() {
        return targetElevationPos;
    }

    public double getTargetRotationPosition() {
        return targetRotationPos;
    }

    public void manualMove(double elevationIncrement, double rotationIncrement){
        double newElevationTarget = elevationIncrement + targetElevationPos;
        double newRotationTarget = rotationIncrement + targetRotationPos;

        if (((newElevationTarget >= MIN_ELEVATION) && (newElevationTarget <= MAX_ELEVATION)) &&
            ((newRotationTarget >= MIN_ROTATION) && (newRotationTarget <= MAX_ROTATION))) {
                setElevationRotationPos(newElevationTarget, newRotationTarget, true);
        } else if (RobotContainer.manualOverride) {
            setElevationRotationPos(newElevationTarget, newRotationTarget, true);
        }
    }

    private void setTargetElevation(double targetElevation, double FFVoltage) {
        setElevationRotationPos(targetElevation, getRotationPos(), FFVoltage);
    }
    private void setTargetElevation(double targetElevation) {
        setElevationRotationPos(targetElevation, getRotationPos());
    }
}
