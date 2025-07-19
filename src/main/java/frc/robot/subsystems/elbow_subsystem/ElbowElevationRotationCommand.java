// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elbow_subsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElbowElevationRotationCommand extends Command {
  double elevation;
  double rotation; 
  private boolean atPosition = false;

  private static final double TOLERANCE = 1.0;
  public ElbowElevationRotationCommand(double elevation, double rotation,ElbowSubsystem elbowSubsystem) {
    this.elevation = elevation;
    this.rotation = rotation;
    this.addRequirements(elbowSubsystem);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("StartingElevate");
    ElbowSubsystem.setElevationRollPos(elevation, rotation, true);
    atPosition = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(ElbowSubsystem.leftElbowEncoder.getPosition() - ElbowSubsystem.leftMotorTargetPos) < TOLERANCE &&
     Math.abs(ElbowSubsystem.rightElbowEncoder.getPosition() - ElbowSubsystem.rightMotorTargetPos) < TOLERANCE){
      atPosition=true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atPosition;
  }
}