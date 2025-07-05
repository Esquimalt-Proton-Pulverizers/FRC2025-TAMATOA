// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elbow_subsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElbowElevationCommmand extends Command {
  double positionRevolutions =0;  
  private ElbowSubsystem elbowSubsystem; 
  private boolean atPosition = false;

  public ElevatorToPosCommand(double positionRevolutions,ElbowSubsystem elbowSubsystem) {
    this.positionRevolutions = positionRevolutions;
    this.elbowSubsystem = elbowSubsystem;
    this.addRequirements(elbowSubsystem);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("StartingElevatorMove");
    elevatorSubsystem.setTargetPosition(positionRevolutions);
    atPosition = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(elevatorSubsystem.elevatorEncoder.getPosition()-positionRevolutions)<1.0){
      atPosition=true;

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
}