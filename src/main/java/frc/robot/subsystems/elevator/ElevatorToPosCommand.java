// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;

/** 
 * An example command that uses an example subsystem. 
 */
public class ElevatorToPosCommand extends Command {
  double targetPositionInches;
  private double startPosition;
  private ElevatorSubsystem elevatorSubsystem; 
  private boolean atPosition = false;
  private final double MAX_VELOCITY = 5.0; // Max speed in inches/sec
  private final double MAX_ACCELERATION = 2.0; // Max acceleration in inches/sec^2
  private double targetDistance = 0.0; // Target distance for the elevator
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
      currentTime = 0.0; // Reset current time
      System.out.println("StartingElevatorMove");
      targetDistance = startPosition - targetPositionInches;
      trapezoidalMotionProfile = TrapezoidalMotionProfile.generateProfile(MAX_VELOCITY, MAX_ACCELERATION, targetDistance);
      atPosition = false;
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      
      // Increment the elapsed time
      currentTime += PERIOD;

    // Determine the current phase of the motion profile
    if (currentTime <= trapezoidalMotionProfile.tAccel) {
        // Acceleration phase
        deltaPosition = 0.5 * MAX_ACCELERATION * currentTime * currentTime;
    } else if (currentTime <= trapezoidalMotionProfile.tAccel + trapezoidalMotionProfile.tConst) {
        // Constant velocity phase
        double tConstStart = trapezoidalMotionProfile.tAccel;
        deltaPosition = (0.5 * MAX_ACCELERATION * tConstStart * tConstStart) +
                         (MAX_VELOCITY * (currentTime - tConstStart));
    } else if (currentTime <= trapezoidalMotionProfile.tTotal) {
        // Deceleration phase
        double tDecelStart = trapezoidalMotionProfile.tAccel + trapezoidalMotionProfile.tConst;
        double tDecel = currentTime - tDecelStart;
        deltaPosition = (0.5 * MAX_ACCELERATION * trapezoidalMotionProfile.tAccel * trapezoidalMotionProfile.tAccel) +
                         (MAX_VELOCITY * trapezoidalMotionProfile.tConst) +
                         (MAX_VELOCITY * tDecel - 0.5 * MAX_ACCELERATION * tDecel * tDecel);
    } else {
        // Motion profile complete
        deltaPosition = targetPositionInches;
        atPosition = true;
    }

    // Command the elevator subsystem to move to the target position
    if (targetDistance < 0) {
      elevatorSubsystem.setTargetPosition(startPosition - deltaPosition);
    }
    else {
      elevatorSubsystem.setTargetPosition(startPosition + deltaPosition);
    }
    if (Math.abs(elevatorSubsystem.elevatorEncoder.getPosition()-targetPositionInches)<1.0){
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
  public static MotionProfileResult generateProfile(double vMax, double aMax, double dTarget) {
      double tAccel = vMax / aMax;
      double dAccel = 0.5 * aMax * tAccel * tAccel;

      double vPeak = vMax;
      double tConst = 0;
      double dConst = 0;

      // Check if the profile is triangular
      if (2 * dAccel >= Math.abs(dTarget)) {
          vPeak = Math.sqrt(aMax * Math.abs(dTarget));
          tAccel = vPeak / aMax;
          dAccel = 0.5 * aMax * tAccel * tAccel;
      } else {
          dConst = Math.abs(dTarget) - 2 * dAccel;
          tConst = dConst / vMax;
      }

      double tTotal = 2 * tAccel + tConst;

      return new MotionProfileResult(tAccel, tConst, tTotal, vPeak);
  }
  // Helper class to store the result
  public static class MotionProfileResult {
      public final double tAccel;
      public final double tConst;
      public final double tTotal;
      public final double vPeak;

      public MotionProfileResult(double tAccel, double tConst, double tTotal, double vPeak) {
          this.tAccel = tAccel;
          this.tConst = tConst;
          this.tTotal = tTotal;
          this.vPeak = vPeak;
      }
  }
}
}