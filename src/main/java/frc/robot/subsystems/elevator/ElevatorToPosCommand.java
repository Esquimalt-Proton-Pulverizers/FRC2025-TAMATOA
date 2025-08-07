// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;

/** 
 * An example command that uses an example subsystem. 
 */
public class ElevatorToPosCommand extends Command {
  double targetPositionInches;
  private double startPosition;
  private ElevatorSubsystem elevatorSubsystem; 
  private boolean atPosition = false;
  private final double MAX_VELOCITY = 25.0; // Max speed in inches/sec
  private final double MAX_ACCELERATION = 15.0; // Max acceleration in inches/sec^2
  private final double MAX_DECELERATION = 3.0; // Max deceleration in inches/sec^2
  private final double kG= 0.26; // Max deceleration in inches/sec^2
  private final double kV = 0.003; // Feedforward gain for velocity
  private final double kA = 0.002; // Feedforward gain for acceleration
  private double targetDistance; // Target distance for the elevator
  private TrapezoidalMotionProfile.MotionProfileResult trapezoidalMotionProfile;
  private double currentTime = 0.0; // Current time in seconds
  private double deltaPosition = 0.0; // Current target position for the elevator
  private final double PERIOD = 0.02; // 20ms periodic update (typical for FRC

  
  public ElevatorToPosCommand(double targetPositionInches,ElevatorSubsystem elevatorSubsystem) {
    this.targetPositionInches = targetPositionInches;
    this.elevatorSubsystem = elevatorSubsystem;
    this.addRequirements(elevatorSubsystem);
  }
    
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPosition = elevatorSubsystem.getPosition(); // Get the current position of the elevator
    targetDistance = targetPositionInches - startPosition;
    if (Math.abs(targetDistance) < 0.3) { // Tolerance for "already at position"
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
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetVelocity = 0.0;
    double targetAcceleration = 0.0;
    if (atPosition){
      elevatorSubsystem.setTargetPosition(targetPositionInches);
      return;
    }
    
    // Increment the elapsed time
    currentTime += PERIOD;

    // Determine the current phase of the motion profile
    if (currentTime <= trapezoidalMotionProfile.tAccel) {
      // Acceleration phase
      deltaPosition = 0.5 * MAX_ACCELERATION * currentTime * currentTime;
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
                      (MAX_VELOCITY * tSpentDecel - 0.5 * MAX_DECELERATION * tSpentDecel * tSpentDecel);
      targetAcceleration = -MAX_DECELERATION;
      targetVelocity = MAX_VELOCITY - MAX_DECELERATION * tSpentDecel;
    } else {
      // Motion profile complete
      deltaPosition = Math.abs(targetDistance);
      atPosition = true;
    }

    // Command the elevator subsystem to move to the target position
    if (targetDistance < 0) {
      double FFVoltage = -kV * targetVelocity + -kA * targetAcceleration + kG;
      //elevatorSubsystem.setTargetPosition(startPosition - deltaPosition);
      elevatorSubsystem.elevatorClosedLoopController.setReference(startPosition - deltaPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, FFVoltage);
    }
    else {
      double FFVoltage = kV * targetVelocity + kA * targetAcceleration + kG;
      elevatorSubsystem.elevatorClosedLoopController.setReference(startPosition + deltaPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, FFVoltage);
    }
    if (Math.abs(elevatorSubsystem.elevatorEncoder.getPosition()-targetPositionInches)<.2){
      atPosition=true;
      elevatorSubsystem.setTargetPosition(targetPositionInches);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

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