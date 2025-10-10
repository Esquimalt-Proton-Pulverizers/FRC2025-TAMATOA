package frc.robot.subsystems.scoring_subsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.RobotModes;
import frc.robot.subsystems.limelight.LimelightHelpers;
import frc.robot.subsystems.scoring_subsystem.elevator.ElevatorSubsystem;

public class ManualScoringControlCommand extends Command {
    private ScoringSubsystem scoringSubsystem;
    private RobotContainer robotContainer;

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
    private final double DIFFERENTIAL_LIFT_MULTIPLIER = 1.0;
    private final double WRIST_MULTIPLER = 2.0;

    private final double ELEVATOR_MAX_POS = ElevatorSubsystem.MAX_ELEVATION - 0;
    private final double ELEVATOR_MIN_POS = ElevatorSubsystem.MIN_ELEVATION + 1;
    private final double DIFFERENTIAL_MAX_POS = -7;
    private final double DIFFERENTIAL_MIN_POS = -130;

    private boolean elevatorMoved, differentialMoved, wristMoved;



    public ManualScoringControlCommand(RobotContainer robotContainer, ScoringSubsystem scoringSubsystem,
        DoubleSupplier elevatorControlAxis,
        DoubleSupplier differentialControlAxis,
        DoubleSupplier wristControlAxis) {
        
        this.scoringSubsystem = scoringSubsystem;
        this.robotContainer = robotContainer;
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
        if (!(robotContainer.getRobotMode()  == RobotModes.ManualMoveMode)) targetElevatorPosition = MathUtil.clamp(targetElevatorPosition, ELEVATOR_MIN_POS, ELEVATOR_MAX_POS);

        if (differentialControlAxis.getAsDouble() != 0) {
            targetDifferentialPosition += (differentialControlAxis.getAsDouble() * DIFFERENTIAL_LIFT_MULTIPLIER);
            differentialMoved = true;
        } else if (differentialMoved) {
            targetDifferentialPosition = scoringSubsystem.getDifferentialSubsystem().getElevationPos();
            differentialMoved = false;
        }
        if (!(robotContainer.getRobotMode()  == RobotModes.ManualMoveMode)) targetDifferentialPosition = MathUtil.clamp(targetDifferentialPosition, DIFFERENTIAL_MIN_POS, DIFFERENTIAL_MAX_POS);

        if (wristControlAxis.getAsDouble() != 0) {
            if ((robotContainer.getRobotMode()  == RobotModes.ManualMoveMode)) {
                targetElevatorPosition = MathUtil.clamp(targetElevatorPosition, ELEVATOR_MIN_POS, ELEVATOR_MAX_POS);
                targetWristPosition +=  wristControlAxis.getAsDouble() * WRIST_MULTIPLER;
            }
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