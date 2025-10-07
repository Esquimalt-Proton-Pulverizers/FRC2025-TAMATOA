package frc.robot.subsystems.scoring_subsystem.differential;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scoring_subsystem.ScoringSubsystem;

public class WristFlipCommand extends Command {

    private ScoringSubsystem scoringSubsystem;

    private double currElevationPosition;

    private double POSITION_TOLERANCE = 10;

    private boolean atPosition = false;
    private boolean orientationNormal;
    private boolean unsafeToFlip = true;

    public WristFlipCommand(ScoringSubsystem scoringSubsystem) {
        this.scoringSubsystem = scoringSubsystem;

        addRequirements(scoringSubsystem);
    }

    @Override
    public void initialize() {
        currElevationPosition = scoringSubsystem.getDifferentialSubsystem().getElevationPos();
        if (-90 < currElevationPosition && currElevationPosition <-30) {// don't do anything unless safe to flip
            unsafeToFlip = false;
            System.out.println("1");
            if (Math.abs(scoringSubsystem.getDifferentialSubsystem().getRotationPos() - 0)  <= POSITION_TOLERANCE) {
                System.out.println("2");
                orientationNormal = true;
            } else if (Math.abs(scoringSubsystem.getDifferentialSubsystem().getRotationPos() - 180)  <= POSITION_TOLERANCE) {
                System.out.println("3");
                orientationNormal = false;
            }
        } else {
            System.out.println("Wrist not safe to flip");
            unsafeToFlip = true;
        } 
    }
    @Override
    public void execute() {
        if (!unsafeToFlip){
            if (orientationNormal) {
                scoringSubsystem.getDifferentialSubsystem().setElevationRotationPos(currElevationPosition, 180);
            } else if (!orientationNormal) {
                scoringSubsystem.getDifferentialSubsystem().setElevationRotationPos(currElevationPosition, 0);
            }
        }
    }
}

