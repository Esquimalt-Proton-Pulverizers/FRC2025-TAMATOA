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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElbowSubsystem extends SubsystemBase{
    public static final double START_POS = 0;
    public static final double CORAL_INTAKE = 0.4;
    public static final double HORIZONTAL = 0.25;

    private Timer timer = new Timer();

    protected SparkMax leftElbowMotor = new SparkMax(2, MotorType.kBrushless);
    protected SparkMax rightElbowMotor = new SparkMax(3, MotorType.kBrushless);

    public SparkMaxConfig leftConfig = new SparkMaxConfig();
    public SparkClosedLoopController leftElbowClosedLoopController = leftElbowMotor.getClosedLoopController();
    public RelativeEncoder leftElbowEncoder = leftElbowMotor.getEncoder();

    public RelativeEncoder rightElbowEncoder = rightElbowMotor.getEncoder();
    public SparkMaxConfig rightConfig = new SparkMaxConfig();
    public SparkClosedLoopController rightElbowClosedLoopController = leftElbowMotor.getClosedLoopController();

    public ElbowSubsystem() {
        timer.start();

    leftConfig.encoder.positionConversionFactor(1/1.785)
    .velocityConversionFactor(1);
    leftConfig.smartCurrentLimit(1,8,50);

    leftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.1).i(0.000001).d(0.0000)
    .outputRange(-.2, .6, ClosedLoopSlot.kSlot0);
    // Set PID values for velocity control in slot 1
        // .p(0.0001, ClosedLoopSlot.kSlot1)
        // .i(0, ClosedLoopSlot.kSlot1)
        // .d(0, ClosedLoopSlot.kSlot1)
        // .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        // .outputRange(-.4, .6, ClosedLoopSlot.kSlot1);

    leftConfig.closedLoop.maxMotion
    // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(3000)
        .maxAcceleration(8000)
        .allowedClosedLoopError(1).positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        

    leftElbowMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightElbowMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    leftElbowClosedLoopController.setReference(START_POS, SparkMax.ControlType.kPosition); 
    rightElbowClosedLoopController.setReference(START_POS, SparkMax.ControlType.kPosition); 
    }

    @Override
    public void periodic() {
      // Put code here to be run every loop
      if(timer.hasElapsed(2.0)) {
        System.out.println("LeftElbowPos" + leftElbowEncoder.getPosition());
        System.out.println("RightElbowPos" + rightElbowEncoder.getPosition());
        timer.reset();
      }
    }

    public void setElevationPos(double targetPosition) {
        leftElbowClosedLoopController.setReference(targetPosition, ControlType.kPosition);
        rightElbowClosedLoopController.setReference(targetPosition, ControlType.kPosition);
    }

    public void setRollPos(double targetRotation) {
        leftElbowClosedLoopController.setReference(leftElbowEncoder.getPosition() - targetRotation, ControlType.kPosition);
        rightElbowClosedLoopController.setReference(rightElbowEncoder.getPosition() + targetRotation, ControlType.kPosition);
    }
}
