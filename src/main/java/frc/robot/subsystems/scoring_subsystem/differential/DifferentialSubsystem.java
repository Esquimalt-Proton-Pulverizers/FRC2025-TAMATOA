package frc.robot.subsystems.scoring_subsystem.differential;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DifferentialSubsystem extends SubsystemBase{
    private final static double START_POS_ELEVATION = 0.0;
    private final static double START_POS_ROTATION  = 0.0;

    // private final static double HORIZONTAL_POS_ELEVATION = -90.0;
    // private final static double HORIZONTAL_POS_ROTATION  =  90.0;
    // private final static double CORAL_COMPENSATION       =  45.0;

    // // private final static double MIN_ELEVATION =    0.0;
    // // private final static double MIN_ROTATION  =    0.0;
    // // private final static double MAX_ELEVATION = -100.0;
    // // private final static double MAX_ROTATION  =  100.0;

    private final static double[] HOMING_POS = {START_POS_ELEVATION - 5.0, START_POS_ROTATION};
    private final static double[] INTAKE_POS = {-98.0, 90.0};
    private final static double[] LOW_POS    = {-26.0, 90.0};
    private final static double[] MIDS_POS   = {-51.5,  0.0};
    private final static double[] HIGH_POS   = {-51.5,  0.0};
    private final static double[] CORAL_POS  = {-26.0, 90.0};

    private Timer timer = new Timer();

    private static boolean hasBeenInitialized = false;

    private static SparkMax leftElbowMotor  = new SparkMax(2, MotorType.kBrushless);
    private static SparkMax rightElbowMotor = new SparkMax(3, MotorType.kBrushless);

    private SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

    protected SparkClosedLoopController leftElbowClosedLoopController = leftElbowMotor.getClosedLoopController();
    protected static  RelativeEncoder leftElbowEncoder = leftElbowMotor.getEncoder();
    
    protected static RelativeEncoder rightElbowEncoder = rightElbowMotor.getEncoder();
    protected SparkClosedLoopController rightElbowClosedLoopController = rightElbowMotor.getClosedLoopController();

    private double ELBOW_MOTORS_GEAR_RATIO = 360/48.0 ;

    protected double leftMotorTargetPos = 0;
    protected double rightMotorTargetPos = 0;

    private static double targetElevationPos;
    private static double targetRotationPos;

    private static final double MIN_ELEVATION = -105.0;
    private static final double MAX_ELEVATION =    0.0;
    private static final double MIN_ROTATION =  -180.0;
    private static final double MAX_ROTATION =   180.0;


    public DifferentialSubsystem() {
        timer.start();

        leftMotorConfig.encoder.positionConversionFactor(ELBOW_MOTORS_GEAR_RATIO)
            .velocityConversionFactor(1);
        leftMotorConfig.smartCurrentLimit(30,20,50);
        double upPIDLimit = 0.4;
        double downPIDLimit = 0.4;
        
        //Full Power PID slot
        leftMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.1).d(0.0000)
            .outputRange(-upPIDLimit, downPIDLimit, ClosedLoopSlot.kSlot0);
        //Half Power PID slot
        leftMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.1, ClosedLoopSlot.kSlot1).d(0.005, ClosedLoopSlot.kSlot1)
            .outputRange(-upPIDLimit/2, downPIDLimit/2, ClosedLoopSlot.kSlot1);

        rightMotorConfig.encoder.positionConversionFactor(ELBOW_MOTORS_GEAR_RATIO);
        rightMotorConfig.smartCurrentLimit(30,20,50);
        rightMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.1).d(0.0000)
            .outputRange(-downPIDLimit, upPIDLimit, ClosedLoopSlot.kSlot0);
        rightMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.1, ClosedLoopSlot.kSlot1).d(0.005, ClosedLoopSlot.kSlot1)
            .outputRange(-downPIDLimit/2, upPIDLimit/2, ClosedLoopSlot.kSlot1);
           

        leftElbowMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightElbowMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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
        System.out.println("-----------------------");
        System.out.println("Elbow Elevation: " + getElevationPos());
        System.out.println("Elbow Rotation: " + getRotationPos());
        timer.reset();
      }
    }

    public void setElevationRotationPos(double elevation, double rotation) {
        targetElevationPos = elevation;
        targetRotationPos = rotation;

        leftMotorTargetPos = -targetElevationPos + targetRotationPos;
        rightMotorTargetPos = targetElevationPos + targetRotationPos;

        double ff = calculateFF(elevation);
        leftElbowClosedLoopController.setReference(leftMotorTargetPos, ControlType.kPosition, ClosedLoopSlot.kSlot0,-ff);
        rightElbowClosedLoopController.setReference(rightMotorTargetPos, ControlType.kPosition, ClosedLoopSlot.kSlot0,ff);
    }
    public void setElevationRotationPos(double elevation, double rotation, boolean slowMode) {
        targetElevationPos = elevation;
        targetRotationPos = rotation;

        leftMotorTargetPos = -targetElevationPos + targetRotationPos;
        rightMotorTargetPos = targetElevationPos + targetRotationPos;

        double ff = calculateFF(elevation);
        if (slowMode) {
            leftElbowClosedLoopController.setReference(leftMotorTargetPos, ControlType.kPosition, ClosedLoopSlot.kSlot1, -ff);
            rightElbowClosedLoopController.setReference(rightMotorTargetPos, ControlType.kPosition, ClosedLoopSlot.kSlot1, ff);
        } else {
            setElevationRotationPos(elevation, rotation);
        }
    }
    private double calculateFF(double elevation){
        return Math.cos(Math.toRadians(-elevation))*0.1;//0.3 would be the power required to hold the arm at 90 degrees horizontally
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
}
