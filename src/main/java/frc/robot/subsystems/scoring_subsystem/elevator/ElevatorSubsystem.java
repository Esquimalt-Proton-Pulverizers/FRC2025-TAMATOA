package frc.robot.subsystems.scoring_subsystem.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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

  private static double elevatorTargetPosition;

  public static final double MIN_ELEVATION =  0.0;
  public static final double MAX_ELEVATION = 62.0;

  // Add a timer object
  private Timer timer = new Timer();

  // Elevator Motor Config
  protected static SparkMax elevatorMotor = new SparkMax(1, MotorType.kBrushless);
  protected SparkMaxConfig elevatorConfig = new SparkMaxConfig();
  private SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();
  public RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  
 
  public ElevatorSubsystem() {
    timer.start();
    elevatorConfig.encoder.positionConversionFactor(1 / 1.347)
      .velocityConversionFactor(1);
    elevatorConfig.smartCurrentLimit(8,8,50);
    elevatorConfig.idleMode(IdleMode.kCoast);

    elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(.1).i(0.00000).d(0.0000) // 0.01
      .outputRange(-.5, .7, ClosedLoopSlot.kSlot0);
  
    elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    elevatorClosedLoopController.setReference(0, ControlType.kVoltage);    
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
    setTargetPosition(targetPosition,0);
  }
  protected void setTargetPosition(double targetPosition, double FFVoltage){
    elevatorTargetPosition = targetPosition;
    elevatorClosedLoopController.setReference(elevatorTargetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, FFVoltage);
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