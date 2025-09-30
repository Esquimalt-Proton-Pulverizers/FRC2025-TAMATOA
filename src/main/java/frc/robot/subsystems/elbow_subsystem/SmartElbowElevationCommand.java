package frc.robot.subsystems.elbow_subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class SmartElbowElevationCommand extends Command {
  double targetElevation;
  double startElevation;
  private ElbowSubsystem elbowSubsystem;
  private ElevatorSubsystem elevatorSubsystem;  
  private boolean atPosition = false;
  private final double TOLERANCE = 2.0; // Tolerance for position check
  private final double MAX_VELOCITY = 45.0; // Max speed in deg/sec
  private final double MAX_ACCELERATION = 35.0; // Max acceleration in deg/sec^2
  private final double MAX_DECELERATION = 35.0; // Max deceleration in deg/sec^2
  private final double GRAVITY_CONSTANT = 0;//0.18; // Feedforward gain for gravity
  private static double kV = 0;// 0.20; // Feedforward gain for velocity
  private static double kA =  0;//0.03; // Feedforward gain for acceleration
  // private static double kG = 0.0; // Feedforward gain for gravity
  private double targetDistance; // Target distance for the elevator
  private TrapezoidalMotionProfile.MotionProfileResult trapezoidalMotionProfile;
  private double currentTime = 0.0; // Current time in seconds
  private double deltaPosition = 0.0; // Current target position for the elevator
  private final double PERIOD = 0.02; // 20ms periodic update (typical for FRC)

  public SmartElbowElevationCommand(double targetElevation, ElbowSubsystem elbowSubsystem, ElevatorSubsystem elevatorSubsystem) {
      this.targetElevation = targetElevation;
      this.elbowSubsystem = elbowSubsystem;
      this.elevatorSubsystem = elevatorSubsystem;
      this.addRequirements(elbowSubsystem);
      this.addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {

    // Update kV and kA from SmartDashboard values
    // kV = SmartDashboard.getNumber("Elevator kV", kV);
    // kA = SmartDashboard.getNumber("Elevator kA", kA);
    double leftMotorPos = ElbowSubsystem.leftElbowMotor.getEncoder().getPosition();
    double rightMotorPos = ElbowSubsystem.rightElbowMotor.getEncoder().getPosition();
    startElevation = (rightMotorPos - leftMotorPos) / 2.0;
    targetDistance = targetElevation - startElevation;
    if (Math.abs(targetDistance) < TOLERANCE) { // Tolerance for "already at position"
      atPosition = true;
      System.out.println("SkippingElevatorMove");
      return;
    }else{
      currentTime = 0.0; // Reset current time
      System.out.println("StartingElevatorMove");
      trapezoidalMotionProfile = TrapezoidalMotionProfile.generateProfile(MAX_DECELERATION, MAX_VELOCITY, MAX_ACCELERATION, targetDistance);
      atPosition = false;
    }
  }
  @Override
  public void execute() {
    // double leftMotorPos = ElbowSubsystem.leftElbowMotor.getEncoder().getPosition();
    // double rightMotorPos = ElbowSubsystem.rightElbowMotor.getEncoder().getPosition();
    // double newElevation = (rightMotorPos - leftMotorPos) / 2.0;
    double targetVelocity = 0.0;
    double targetAcceleration = 0.0;
    double kG = 0;// TODO move GRAVITY_CONSTANT * Math.cos(Math.toRadians(newElevation));
    if (atPosition){
      // elbowSubsystem.setTargetElevation(targetElevation,kG);
      System.out.println("atPos "+ targetElevation);
      return;
    }
    // Increment the elapsed time
    currentTime += PERIOD;

    // Determine the current phase of the motion profile
    if (currentTime <= trapezoidalMotionProfile.tAccel) {
      // Acceleration phase
      deltaPosition = 0.5 * MAX_ACCELERATION * currentTime * currentTime;
      // System.out.println("accel dP = "+ deltaPosition);
      targetAcceleration = MAX_ACCELERATION;
      targetVelocity = MAX_ACCELERATION * currentTime;
    } else if (currentTime <= trapezoidalMotionProfile.tAccel + trapezoidalMotionProfile.tConst) {
      // Constant velocity phase
      double tConstStart = trapezoidalMotionProfile.tAccel;
      deltaPosition = (0.5 * MAX_ACCELERATION * tConstStart * tConstStart) +
                      (MAX_VELOCITY * (currentTime - tConstStart));
      targetAcceleration = 0;
      targetVelocity = MAX_VELOCITY;
    } else if (currentTime <= trapezoidalMotionProfile.tTotal) {
      // Deceleration phase
      double tDecelStart = trapezoidalMotionProfile.tAccel + trapezoidalMotionProfile.tConst;
      double tSpentDecel = currentTime - tDecelStart;
      deltaPosition = (0.5 * MAX_ACCELERATION * trapezoidalMotionProfile.tAccel * trapezoidalMotionProfile.tAccel) +
                      (MAX_VELOCITY * trapezoidalMotionProfile.tConst) +
                      (trapezoidalMotionProfile.vPeak * tSpentDecel - 0.5 * MAX_DECELERATION * tSpentDecel * tSpentDecel);
      targetAcceleration = -MAX_DECELERATION;
      targetVelocity = trapezoidalMotionProfile.vPeak - MAX_DECELERATION * tSpentDecel;
      // System.out.println("decel dP = "+ deltaPosition);
      // System.out.println("tspentdecel = "+ tSpentDecel);
    } else {
      // Motion profile complete
      deltaPosition = Math.abs(targetDistance);
      atPosition = true;
    }

    // Command the elevator subsystem to move to the target position
    if (targetDistance < 0) {
      double FFVoltage = -kV * targetVelocity + -kA * targetAcceleration - kG;
      elbowSubsystem.setTargetElevation(startElevation - deltaPosition, FFVoltage);
    }
    else {
      double FFVoltage = kV * targetVelocity + kA * targetAcceleration + kG;
      elbowSubsystem.setTargetElevation(startElevation + deltaPosition, FFVoltage);
      //System.out.println("deltaP = "+ deltaPosition);
    }
    if (Math.abs(elbowSubsystem.getElevationPos()-targetElevation)<.2){
      atPosition=true;
      elbowSubsystem.setTargetElevation(targetElevation,kG);
    }
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atPosition;
  }
  public class TrapezoidalMotionProfile {
    public static MotionProfileResult generateProfile(double deMax, double vMax, double aMax, double dTarget) {
        double tAccel = vMax / aMax;
        double dAccel = 0.5 * aMax * tAccel * tAccel;
  
        double tDecel = vMax / deMax;
        double dDecel = 0.5 * deMax * tDecel * tDecel;
  
        double vPeak = vMax;
        double tConst = 0;
        double dConst = 0;
  
        // Check if the profile is triangular
        if (dDecel + dAccel >= Math.abs(dTarget)) {
          vPeak = Math.sqrt((2 * Math.abs(dTarget)) / ((1 / aMax) + (1 / deMax)));;
          tAccel = vPeak / aMax;
          dAccel = 0.5 * aMax * tAccel * tAccel;
          tDecel = vPeak / deMax;
          dDecel = 0.5 * deMax * tDecel * tDecel;
      } else {
          dConst = Math.abs(dTarget) - (dAccel + dDecel);
          tConst = dConst / vMax;
      }
  
        double tTotal = tDecel + tAccel + tConst;
  
        return new MotionProfileResult(tAccel, tDecel, tConst, tTotal, vPeak);
    }
  // Helper class to store the result
  public static class MotionProfileResult {
    public final double tAccel;
    public final double tDecel;
    public final double tConst;
    public final double tTotal;
    public final double vPeak;

    public MotionProfileResult(double tAccel, double tDecel, double tConst, double tTotal, double vPeak) {
        this.tAccel = tAccel;
        this.tDecel = tDecel;
        this.tConst = tConst;
        this.tTotal = tTotal;
        this.vPeak = vPeak;
    }
  }
 }
}

