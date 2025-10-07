package frc.robot.subsystems.scoring_subsystem.elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class DecrementElevatorCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private double currElevatorPosition;
    private double targetElevatorPosition;
  
    private final double DECREMENT_VALUE = 2;
    private final double POSITION_TOLERANCE = 1;
  
    public DecrementElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
      this.elevatorSubsystem = elevatorSubsystem;
  
      addRequirements(elevatorSubsystem);
    }
  
    @Override
    public void initialize() {
      currElevatorPosition = elevatorSubsystem.getPosition();
    }
  
    @Override
    public void execute() {
      currElevatorPosition = elevatorSubsystem.getPosition();
      targetElevatorPosition = currElevatorPosition - DECREMENT_VALUE;
      
      elevatorSubsystem.setTargetPosition(targetElevatorPosition);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setTargetPosition(elevatorSubsystem.getPosition());
    }
  
    @Override
    public boolean isFinished() {
      return Math.abs(currElevatorPosition - targetElevatorPosition) <= POSITION_TOLERANCE;
    }
  }
