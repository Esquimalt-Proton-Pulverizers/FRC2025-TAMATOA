package frc.robot.subsystems.scoring_subsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class ManualScoringControlCommand extends Command {
    private ScoringSubsystem scoringSubsystem;

    private DoubleSupplier elevatorControlAxis;
    private DoubleSupplier differentialControlAxis;
    private DoubleSupplier wristControlAxis; 

    private double currElevatorPosition;
    private double currDifferentialPosition;
    private double currWristPosition;
    
    private double targetElevatorPosition;
    private double targetDifferentialPosition;
    private double targetWristPosition;

    private final double ELEVATOR_LIFT_MULTIPLIER = 3;
    private final double ELEVATOR_LOWER_MULTIPLER = 1.5;
    private final double DIFFERENTIAL_LIFT_MULTIPLIER = 3;
    private final double DIFFERENTIAL_LOWER_MULTIPLER = 1;
    private final double WRIST_CW_MULTIPLER = 5;
    private final double WRIST_CCW_MULTIPLER = 5;

    private final double ELEVATOR_MAX_POS = 59;
    private final double ELEVATOR_MIN_POS = 2;
    private final double DIFFERENTIAL_MAX_POS = -7;
    private final double DIFFERENTIAL_MIN_POS = -130;



    public ManualScoringControlCommand(ScoringSubsystem scoringSubsystem,
        DoubleSupplier elevatorControlAxis,
        DoubleSupplier differentialControlAxis,
        DoubleSupplier wristControlAxis) {
        
        this.scoringSubsystem = scoringSubsystem;
        
        this.elevatorControlAxis = elevatorControlAxis;
        this.differentialControlAxis = differentialControlAxis;
        this.wristControlAxis = wristControlAxis;

        addRequirements(scoringSubsystem);
    }

    @Override
    public void initialize() {
        currElevatorPosition = scoringSubsystem.getElevatorSubsystem().getPosition();
        currDifferentialPosition = scoringSubsystem.getDifferentialSubsystem().getElevationPos();
        currWristPosition = scoringSubsystem.getDifferentialSubsystem().getRotationPos();
    }


    @Override
    public void execute() {
        currElevatorPosition = scoringSubsystem.getElevatorSubsystem().getPosition();
        currDifferentialPosition = scoringSubsystem.getDifferentialSubsystem().getElevationPos();
        currWristPosition = scoringSubsystem.getDifferentialSubsystem().getRotationPos();

        if (elevatorControlAxis.getAsDouble() >= 0) {
            targetElevatorPosition = currElevatorPosition + (elevatorControlAxis.getAsDouble() * ELEVATOR_LIFT_MULTIPLIER);
        }
        else {
            targetElevatorPosition = currElevatorPosition + (elevatorControlAxis.getAsDouble() * ELEVATOR_LOWER_MULTIPLER);
        }
             targetElevatorPosition = MathUtil.clamp(targetElevatorPosition, ELEVATOR_MIN_POS, ELEVATOR_MAX_POS);

        if (differentialControlAxis.getAsDouble() >= 0) {
            targetDifferentialPosition = currDifferentialPosition + (differentialControlAxis.getAsDouble() * DIFFERENTIAL_LIFT_MULTIPLIER);
        }
        else {
            targetDifferentialPosition = currDifferentialPosition + (differentialControlAxis.getAsDouble() * DIFFERENTIAL_LOWER_MULTIPLER);
        }
            targetDifferentialPosition = MathUtil.clamp(targetDifferentialPosition, DIFFERENTIAL_MIN_POS, DIFFERENTIAL_MAX_POS);

        if (wristControlAxis.getAsDouble() >= 0) {
            targetWristPosition = currWristPosition /*+ (wristControlAxis.getAsDouble() * WRIST_CW_MULTIPLER)*/;
        }
        else {
            targetWristPosition = currWristPosition /*+ (wristControlAxis.getAsDouble() * WRIST_CCW_MULTIPLER)*/;
        }

        scoringSubsystem.getDifferentialSubsystem().setElevationRotationPos(targetDifferentialPosition, targetWristPosition);
        scoringSubsystem.getElevatorSubsystem().setTargetPosition(targetElevatorPosition);

        System.out.println("Elevator Control Multipler: " + elevatorControlAxis.getAsDouble());
        System.out.println("Differntial Control Multipler: " + differentialControlAxis.getAsDouble());
        System.out.println("Wrist Control Multipler: " + wristControlAxis.getAsDouble());
    }
}