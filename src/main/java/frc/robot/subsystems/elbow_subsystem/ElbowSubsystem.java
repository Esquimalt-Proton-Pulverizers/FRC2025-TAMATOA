package frc.robot.subsystems.elbow_subsystem;

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

public class ElbowSubsystem extends SubsystemBase{
    
    
    public static final     double[] START_POS      = {0.0,0.0};
    public static final     double[] INTAKE_POS     = {-98.0,89.0};
    public static final     double[] LOW_POS        = {-26.0,90.0};
    public static final     double[] MIDS_POS       = {-51.5,0.0};
    public static final     double[] HIGH_POS       = {-44,0.0};
    private static final    double TOLERANCE        = 1.0;
    private static final    double ELBOW_MOTORS_GEAR_RATIO = 360/48.0 ; //converts to degrees

    private static Timer timer = new Timer();
    private static boolean hasBeenInitialized = false;

    protected static SparkMax leftElbowMotor    = new SparkMax(2, MotorType.kBrushless);
    protected static SparkMax rightElbowMotor   = new SparkMax(3, MotorType.kBrushless);

    private static SparkMaxConfig motorConfig   = new SparkMaxConfig();
    private static SparkClosedLoopController leftElbowClosedLoopController = leftElbowMotor.getClosedLoopController();
    public static  RelativeEncoder leftElbowEncoder = leftElbowMotor.getEncoder();
    
    public static RelativeEncoder rightElbowEncoder = rightElbowMotor.getEncoder();
    private static SparkClosedLoopController rightElbowClosedLoopController = rightElbowMotor.getClosedLoopController();

    

    protected static double elevationTargetPos = 0;
    protected static double rollTargetPos = 0;

    protected static double leftMotorTargetPos = 0;
    protected static double rightMotorTargetPos = 0;

    

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

        leftElbowMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightElbowMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        setElevationRollPos(START_POS[0] , START_POS[1]);
    }

    private static void resetEncoder() {
        leftElbowMotor.getEncoder().setPosition(0);
        rightElbowMotor.getEncoder().setPosition(0);
    }

    protected static void setElevationRollPos(double elevation, double roll) {
        leftMotorTargetPos = -elevation + roll;
        rightMotorTargetPos = elevation + roll;

        leftElbowClosedLoopController.setReference(leftMotorTargetPos, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        rightElbowClosedLoopController.setReference(rightMotorTargetPos, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    protected static void setElevationRollPos(double elevation, double roll, boolean slowMode) {
        leftMotorTargetPos = -elevation + roll;
        rightMotorTargetPos = elevation + roll;

        if (slowMode) {
            leftElbowClosedLoopController.setReference(leftMotorTargetPos, ControlType.kPosition, ClosedLoopSlot.kSlot1);
            rightElbowClosedLoopController.setReference(rightMotorTargetPos, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        } else {
            setElevationRollPos(elevation, roll);
        }
    }

    public static void initialize(){
        if(!hasBeenInitialized) {
            resetEncoder();

            leftElbowEncoder.setPosition(0);
            rightElbowEncoder.setPosition(0);
            hasBeenInitialized = true;
        }
        setElevationRollPos(START_POS[0] , START_POS[1]);
    }

    @Override
    public void periodic() {
      // Put code here to be run every loop
      if(timer.hasElapsed(2.0)) {
        // System.out.println("LeftElbowPos" + leftElbowEncoder.getPosition());
        // System.out.println("RightElbowPos" + rightElbowEncoder.getPosition());

        System.out.println("-----------------------");
        System.out.println("Elbow Elevation: " + getElevationPos());
        System.out.println("Elbow Roll: " + getRollPos());
        timer.reset();
      }
    }



    public double getElevationPos() {
        double leftMotorPos = leftElbowMotor.getEncoder().getPosition();
        double rightMotorPos = rightElbowMotor.getEncoder().getPosition();

        double elevation = (rightMotorPos - leftMotorPos) / 2.0;

        return elevation;
    }

    public double getRollPos() {
        double leftMotorPos = leftElbowMotor.getEncoder().getPosition();
        double rightMotorPos = rightElbowMotor.getEncoder().getPosition();

        double rotation = (rightMotorPos + leftMotorPos) / 2.0;

        return rotation;
    }

    public boolean atPosition(){
        return (Math.abs(leftElbowEncoder.getPosition() - leftMotorTargetPos) < TOLERANCE &&
         Math.abs(rightElbowEncoder.getPosition() - rightMotorTargetPos) < TOLERANCE);
    }

    public Command resetEncoderCommand() {
        return Commands.runOnce(() -> {
            resetEncoder();
        });    
    }

    
}