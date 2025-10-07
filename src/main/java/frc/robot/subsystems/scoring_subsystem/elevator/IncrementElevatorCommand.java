package frc.robot.subsystems.scoring_subsystem.elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class IncrementElevatorCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private double currElevatorPosition;
    private double targetElevatorPosition;
  
    private final double INCREMENT_VALUE = 3;
    private final double POSITION_TOLERANCE = 1;
  
    public IncrementElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
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
      targetElevatorPosition = currElevatorPosition + INCREMENT_VALUE;
      
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
