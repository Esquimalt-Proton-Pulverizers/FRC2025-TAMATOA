package frc.robot.subsystems.elevator;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;



public class ElevatorSubsystem extends SubsystemBase {
  // Position Constants
  public static final double LOCK_POSITION   =  0.0;
  public static final double LOW_POSITION    =  3.0; // Correct Low position value 2.0 
  public static final double LEVEL1_POSITION =  5.0;
  public static final double LEVEL2_POSITION = 16.5;
  public static final double LEVEL3_POSITION = LEVEL2_POSITION + 16.0;
  public static final double LEVEL4_POSITION = 58.0;
  public static final double NET_POSITION    = 50;
  public static final double CORAL_STATION_POSITION = 19.0;

  private static double elevatorTargetPosition;

  public static final double MIN_ELEVATION =  0.0;
  public static final double MAX_ELEVATION = 62.0;

  // Add a timer object
  private Timer timer = new Timer();

  // Elevator Motor Config
  protected static SparkMax elevatorMotor = new SparkMax(1, MotorType.kBrushless);
  protected SparkMaxConfig elevatorConfig = new SparkMaxConfig();
  protected SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();
  public RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  
 
  public ElevatorSubsystem() {
    timer.start();
    elevatorConfig.encoder.positionConversionFactor(1 / 1.347)
      .velocityConversionFactor(1);
      elevatorConfig.smartCurrentLimit(1,8,50);

    elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1).i(0.000001).d(0.0000)
      .outputRange(-.2, .6, ClosedLoopSlot.kSlot0);
      // Set PID values for velocity control in slot 1
      // .p(0.0001, ClosedLoopSlot.kSlot1)
      // .i(0, ClosedLoopSlot.kSlot1)
      // .d(0, ClosedLoopSlot.kSlot1)
      // .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
      // .outputRange(-.4, .6, ClosedLoopSlot.kSlot1);

    elevatorConfig.closedLoop.maxMotion
      // Set MAXMotion parameters for position control. We don't need to pass
      // a closed loop slot, as it will default to slot 0.
      .maxVelocity(3000)
      .maxAcceleration(8000)
      .allowedClosedLoopError(1).positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        
    elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    elevatorClosedLoopController.setReference(LOW_POSITION, SparkMax.ControlType.kPosition);    
  }

  @Override
  public void periodic() {
    // Put code here to be run every loop
    if(timer.hasElapsed(2.0)) {
       System.out.println("Elevator target position"+getTargetPosition());
      // System.out.println("Is inverted: " + !isCompetitionRobot);
      System.out.println("Elevator Level: " + getPosition());
      timer.reset();
    }
  }
  protected void setTargetPosition(double targetPosition){
    elevatorTargetPosition = targetPosition;
    elevatorClosedLoopController.setReference(elevatorTargetPosition, ControlType.kPosition);
  }

  public static void resetEncoder() {
    elevatorMotor.getEncoder().setPosition(0);
  }

  public double getPosition() {
    return elevatorEncoder.getPosition();
  }

  public double getTargetPosition() {
    return elevatorTargetPosition;
  }
  public void manualMove(double distanceIncrement){
    double newTarget = distanceIncrement + elevatorTargetPosition;
    setTargetPosition(newTarget);

    if ((newTarget >= MIN_ELEVATION) && (newTarget <= MAX_ELEVATION)) {
      setTargetPosition(newTarget);
    } else if (RobotContainer.manualOverride) {
      setTargetPosition(newTarget);
    }
  }
}