// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;

/** 
 * An example command that uses an example subsystem. 
 */
public class ElevatorInchUpCommand extends Command { 
  private ElevatorSubsystem elevatorSubsystem; 
  private boolean atPosition = false;
  private double targetPosition = 0.0; // Target position for the elevator
  public ElevatorInchUpCommand(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.addRequirements(elevatorSubsystem);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("StartingElevatorMove");
    targetPosition = elevatorSubsystem.getPosition() + 1.0; // Increment target position by 1 inch
    elevatorSubsystem.setTargetPosition(targetPosition); // Move up by 1 inch
    atPosition = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(elevatorSubsystem.elevatorEncoder.getPosition()-targetPosition)<1.0){
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