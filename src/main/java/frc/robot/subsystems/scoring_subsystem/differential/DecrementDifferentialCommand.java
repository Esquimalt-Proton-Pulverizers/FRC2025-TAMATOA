package frc.robot.subsystems.scoring_subsystem.differential;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scoring_subsystem.ScoringSubsystem;

public class DecrementDifferentialCommand extends Command {
    private ScoringSubsystem scoringSubsystem;
    private double currDifferentialPosition;
    private double targetDifferentialPosition;
    private double currWristPosition;
  
    private final double DECREMENT_VALUE = .5;
    private final double POSITION_TOLERANCE = .25;
  
    public DecrementDifferentialCommand(ScoringSubsystem scoringSubsystem) {
      this.scoringSubsystem = scoringSubsystem;
  
      addRequirements(scoringSubsystem);
    }
  
    @Override
    public void initialize() {
      currDifferentialPosition = scoringSubsystem.getDifferentialSubsystem().getElevationPos();
      currWristPosition = scoringSubsystem.getDifferentialSubsystem().getRotationPos();
    }
  
    @Override
    public void execute() {
      currDifferentialPosition = scoringSubsystem.getDifferentialSubsystem().getElevationPos();
      targetDifferentialPosition = currDifferentialPosition - DECREMENT_VALUE;
      
      scoringSubsystem.getDifferentialSubsystem().setElevationRotationPos(targetDifferentialPosition, currWristPosition);
    }

    @Override
    public void end(boolean interrupted) {
        scoringSubsystem.getDifferentialSubsystem().setElevationRotationPos(scoringSubsystem.getDifferentialSubsystem().getElevationPos(), scoringSubsystem.getDifferentialSubsystem().getRotationPos());
    }
  
    @Override
    public boolean isFinished() {
      return Math.abs(currDifferentialPosition - targetDifferentialPosition) <= POSITION_TOLERANCE;
    }
  }