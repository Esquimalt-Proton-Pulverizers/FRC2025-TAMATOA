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

    private final double ELEVATOR_LIFT_MULTIPLIER = 1.0;
    private final double ELEVATOR_LOWER_MULTIPLER = 1.0;
    private final double DIFFERENTIAL_LIFT_MULTIPLIER = 1.0;
    private final double DIFFERENTIAL_LOWER_MULTIPLER = 1.0;
    private final double WRIST_CW_MULTIPLER = 5;
    private final double WRIST_CCW_MULTIPLER = 5;

    private final double ELEVATOR_MAX_POS = 59;
    private final double ELEVATOR_MIN_POS = 2;
    private final double DIFFERENTIAL_MAX_POS = -7;
    private final double DIFFERENTIAL_MIN_POS = -130;

    private boolean elevatorMoved, differentialMoved, wristMoved;



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
        targetElevatorPosition = currElevatorPosition;
        targetDifferentialPosition = currDifferentialPosition;  
        targetWristPosition = currWristPosition;
    }


    @Override
    public void execute() {

        if (elevatorControlAxis.getAsDouble() != 0) {
            targetElevatorPosition += (elevatorControlAxis.getAsDouble() * ELEVATOR_LIFT_MULTIPLIER);
            elevatorMoved = true;
        } else if (elevatorMoved) {
            targetElevatorPosition = scoringSubsystem.getElevatorSubsystem().getPosition();
            elevatorMoved = false;
        }
        targetElevatorPosition = MathUtil.clamp(targetElevatorPosition, ELEVATOR_MIN_POS, ELEVATOR_MAX_POS);

        if (differentialControlAxis.getAsDouble() != 0) {
            targetDifferentialPosition += (differentialControlAxis.getAsDouble() * DIFFERENTIAL_LIFT_MULTIPLIER);
            differentialMoved = true;
        } else if (differentialMoved) {
            targetDifferentialPosition = scoringSubsystem.getDifferentialSubsystem().getElevationPos();
            differentialMoved = false;
        }
        targetDifferentialPosition = MathUtil.clamp(targetDifferentialPosition, DIFFERENTIAL_MIN_POS, DIFFERENTIAL_MAX_POS);

        if (wristControlAxis.getAsDouble() != 0) {
            targetWristPosition += 0 /*+ (wristControlAxis.getAsDouble() * WRIST_CW_MULTIPLER)*/;
            wristMoved = true;
        } else if (wristMoved){
            targetWristPosition = scoringSubsystem.getDifferentialSubsystem().getRotationPos();
            wristMoved = false;
        }

        scoringSubsystem.getDifferentialSubsystem().setElevationRotationPos(targetDifferentialPosition, targetWristPosition);
        scoringSubsystem.getElevatorSubsystem().setTargetPosition(targetElevatorPosition);

        // System.out.println("Elevator Control Multipler: " + elevatorControlAxis.getAsDouble());
        // System.out.println("Differntial Control Multipler: " + differentialControlAxis.getAsDouble());
        // System.out.println("Wrist Control Multipler: " + wristControlAxis.getAsDouble());
    }
}