// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;

/** 
 * An example command that uses an example subsystem. 
 */
public class ElevatorToPosCommand extends Command {
  double positionRevolutions = 0;  
  private ElevatorSubsystem elevatorSubsystem; 
  private boolean atPosition = false;
  private boolean incrementalMove = false;

  public ElevatorToPosCommand(double positionRevolutions,ElevatorSubsystem elevatorSubsystem) {
    this(positionRevolutions, elevatorSubsystem, false, false);
  }

  public ElevatorToPosCommand(double positionRevolutions,ElevatorSubsystem elevatorSubsystem, boolean incrementalMove, boolean manualOverride) {
    this.positionRevolutions = positionRevolutions;
    this.elevatorSubsystem = elevatorSubsystem;
    this.addRequirements(elevatorSubsystem);
    this.incrementalMove = incrementalMove;
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("StartingElevatorMove");

    if (incrementalMove) {
      positionRevolutions = elevatorSubsystem.getPosition() + positionRevolutions;
      System.out.println("Target Position: " + elevatorSubsystem.getTargetPosition());
      System.out.println("Current Position: " + elevatorSubsystem.getPosition());
      System.out.println("Elevator position revolution: " + positionRevolutions);
    }

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