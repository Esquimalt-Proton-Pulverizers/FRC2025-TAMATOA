// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elbow_subsystem;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;

/** 
 * An example command that uses an example subsystem. 
 * */
public class ElbowElevationRotationCommand extends Command {
  double elevation;
  double rotation; 
  private ElbowSubsystem elbowSubsystem; 
  private boolean atPosition = false;

  private static final double TOLERANCE = 3.0;
  public ElbowElevationRotationCommand(double elevation, double rotation, ElbowSubsystem elbowSubsystem) {
    this(elevation, rotation, elbowSubsystem, false);
  }

  public ElbowElevationRotationCommand(double elevation, double rotation, ElbowSubsystem elbowSubsystem, boolean manualOverride) {
    this.elevation = elevation;
    this.rotation = rotation;
    this.elbowSubsystem = elbowSubsystem;
    this.addRequirements(elbowSubsystem);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("StartingElevate");

    elbowSubsystem.setElevationRotationPos(elevation, rotation, true);
    atPosition = false;

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(ElbowSubsystem.leftElbowEncoder.getPosition() - elbowSubsystem.leftMotorPos)< TOLERANCE && Math.abs(elbowSubsystem.rightElbowEncoder.getPosition() - elbowSubsystem.rightMotorPos) < TOLERANCE){
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