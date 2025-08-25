package frc.robot.subsystems.scoring_subsystem;

import static edu.wpi.first.units.Units.Rotation;

import javax.naming.spi.StateFactory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elbow_subsystem.ElbowElevationRotationCommand;
import frc.robot.subsystems.elbow_subsystem.ElbowSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToPosCommand;

public class ScoringSubsystem {
    State currentState = State.CORAL_GROUND_INTAKE;
    State targetState = State.SCORE_L2;
    double[] targetVals; // 1; Wrist pos, 2; Diff pos, 3; Elevator pos
    double elbElevation;
    double elbRotation;
    double elevation;

    public Command moveArm(State currentState, State targetState, ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem){
        double leftMotorPos = ElbowSubsystem.leftElbowMotor.getEncoder().getPosition();
        double rightMotorPos = ElbowSubsystem.rightElbowMotor.getEncoder().getPosition();

        elbElevation = (rightMotorPos - leftMotorPos) / 2.0;
        elbRotation = (rightMotorPos + leftMotorPos) / 2.0;
        if (currentState == targetState) {
            return new InstantCommand(); // No movement needed
        }

        S seq = getSequence(currentState, targetState);
        switch (targetState)    {
            case CORAL_GROUND_INTAKE:
                targetVals = new double[]{0.0, 0, 0};
            case ALGAE_GROUND_INTAKE:
                targetVals = new double[]{0.0, 0, 0};
            case CORAL_STATION_INTAKE: 
                targetVals = new double[]{0.0, 0, 0};
            case ALGAE_LOLLIPOP_INTAKE:
                targetVals = new double[]{0.0, 0, 0};
            case HOME_CLIMB:
                targetVals = new double[]{0.0, 0, 0};
            case SET_CORAL_POSITION_LEFT:
                targetVals = new double[]{0.0, 0, 0};
            case SET_CORAL_POSITION_RIGHT:
                targetVals = new double[]{0.0, 0, 0};
            case SCORE_L1:
                targetVals = new double[]{0.0, 0, 0};
            case SCORE_L2: 
                targetVals = new double[]{0.0, 0, 0}; 
            case SCORE_L3:
                targetVals = new double[]{0.0, 0, 0};
            case SCORE_L4:  
                targetVals = new double[]{0.0, 0, 0};
            case SCORE_NET:
                targetVals = new double[]{0.0, 0, 0};
            case SCORE_PROCESSOR:
                targetVals = new double[]{0.0, 0, 0};
            case DRIVE_EMPTY:
                targetVals = new double[]{0.0, 0, 0};
            case DRIVE_WITH_CORAL:
                targetVals = new double[]{0.0, 0, 0};
            case DRIVE_WITH_ALGAE:
                targetVals = new double[]{0.0, 0, 0};
            case OTHER:
                targetVals = new double[]{0.0, 0, 0};
                break;
        }
    
        return switch (seq) {
            case WDE -> returnWDECommand(targetVals, elevatorSubsystem, elbowSubsystem);
            case WED -> returnWEDCommand(targetVals, elevatorSubsystem, elbowSubsystem);
            case DWE -> returnDWECommand(targetVals, elevatorSubsystem, elbowSubsystem);
            case DEW -> returnDEWCommand(targetVals, elevatorSubsystem, elbowSubsystem);
            case EDW -> returnEDWCommand(targetVals, elevatorSubsystem, elbowSubsystem);
            case EWD -> returnEWDCommand(targetVals, elevatorSubsystem, elbowSubsystem);
            //case OOO -> returnOther();
            case XXX -> returnWDECommand(targetVals, elevatorSubsystem, elbowSubsystem); // Default to WDE if no movement needed
            
            default -> throw new IllegalStateException("Unexpected sequence: " + seq);
        };
    }
    
    
    
    private Command returnWDECommand(double[] targetVals, ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem) {
        new ElbowElevationRotationCommand(targetVals[0], elbRotation, elbowSubsystem);
        new ElbowElevationRotationCommand(elbElevation, targetVals[1], elbowSubsystem);
        new ElevatorToPosCommand(targetVals[2], elevatorSubsystem);
        return new InstantCommand();
    }

    private Command returnWEDCommand(double[] targetVals, ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem) {
        new ElbowElevationRotationCommand(targetVals[0], elbRotation, elbowSubsystem);
        new ElevatorToPosCommand(targetVals[2], elevatorSubsystem);
        new ElbowElevationRotationCommand(elbElevation,targetVals[1], elbowSubsystem);
        return new InstantCommand();
    }

    private Command returnDWECommand(double[] targetVals, ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem) {
        new ElbowElevationRotationCommand(elbElevation, targetVals[1], elbowSubsystem);
        new ElbowElevationRotationCommand(targetVals[0], elbRotation, elbowSubsystem);
        new ElevatorToPosCommand(targetVals[2], elevatorSubsystem);
        return new InstantCommand();
    }

    private Command returnDEWCommand(double[] targetVals, ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem) {
        new ElbowElevationRotationCommand(elbElevation, targetVals[1], elbowSubsystem);
        new ElevatorToPosCommand(targetVals[2], elevatorSubsystem);
        new ElbowElevationRotationCommand(targetVals[0], elbRotation, elbowSubsystem);
        return new InstantCommand();
    }

    private Command returnEDWCommand(double[] targetVals, ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem) {
        new ElevatorToPosCommand(targetVals[2], elevatorSubsystem);
        new ElbowElevationRotationCommand(elbElevation, targetVals[1], elbowSubsystem);
        new ElbowElevationRotationCommand(targetVals[0], elbRotation, elbowSubsystem);
        return new InstantCommand();
    }

    private Command returnEWDCommand(double[] targetVals, ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem) {
        new ElevatorToPosCommand(targetVals[2], elevatorSubsystem);
        new ElbowElevationRotationCommand(targetVals[0], elbRotation, elbowSubsystem);
        new ElbowElevationRotationCommand(elbElevation, targetVals[1], elbowSubsystem);
        return new InstantCommand();
    }






    public enum State {
        CORAL_GROUND_INTAKE,
        ALGAE_GROUND_INTAKE,
        CORAL_STATION_INTAKE,
        ALGAE_LOLLIPOP_INTAKE,
        HOME_CLIMB,
        SET_CORAL_POSITION_LEFT,
        SET_CORAL_POSITION_RIGHT,
        SCORE_L1,
        SCORE_L2,
        SCORE_L3,
        SCORE_L4,
        SCORE_NET,
        SCORE_PROCESSOR,
        DRIVE_EMPTY,
        DRIVE_WITH_CORAL,
        DRIVE_WITH_ALGAE,
        OTHER // wrist/elbow/elevator block if needed
    }

    public enum S {//W is wrist, D is differential (sometimes called elbow), and E is elevator
        //x is default, O is other, and U is still unknown and needs to be solved
        //they are three letters because Colin is OCD and the matrix is easier to read
        WDE("1"), WED("2"), DWE("3"), DEW("4"), EDW("5"), EWD("6"),
        OOO("O"), UUU("U"), XXX("x");

        private final String label;
        S(String l) { this.label = l; }
        @Override 
        public String toString() { return label; }
    }

    // Matrix layout: rows = from state, cols = to state
    private static final S[][] matrix = {
        // CG_IN AG_IN CS_IN AL_LO HOME SET_L SET_R L1 L2 L3 L4 NET PROC DR_EMP DR_COR DR_ALG OTHER
        /*CG_IN*/   {S.XXX, S.EDW, S.XXX, S.XXX, S.WDE, S.OOO, S.OOO, S.XXX, S.XXX, S.XXX, S.XXX, S.WDE, S.XXX, S.XXX, S.XXX, S.WDE, S.OOO},
        /*AG_IN*/   {S.WDE, S.XXX, S.XXX, S.XXX, S.WDE, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.OOO, S.WDE, S.XXX, S.XXX, S.XXX, S.XXX},
        /*CS_IN*/   {S.WDE, S.WDE, S.XXX, S.WED, S.WDE, S.WDE, S.WDE, S.WED, S.XXX, S.WED, S.WED, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX},
        /*AL_LO*/   {S.UUU, S.UUU, S.XXX, S.UUU, S.UUU, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.UUU, S.UUU, S.XXX, S.XXX, S.XXX, S.XXX},
        /*HOME*/    {S.DWE, S.DWE, S.DWE, S.UUU, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX},
        /*SET_L*/   {S.UUU, S.UUU, S.XXX, S.XXX, S.WDE, S.OOO, S.XXX, S.WDE, S.XXX, S.XXX, S.EDW, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX},
        /*SET_R*/   {S.UUU, S.UUU, S.XXX, S.XXX, S.OOO, S.XXX, S.OOO, S.WDE, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX},
        /*L1*/      {S.WDE, S.WDE, S.XXX, S.XXX, S.WED, S.WDE, S.WDE, S.XXX, S.XXX, S.WED, S.WED, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX},
        /*L2*/      {S.WDE, S.WDE, S.XXX, S.XXX, S.XXX, S.WDE, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX},
        /*L3*/      {S.WDE, S.WDE, S.XXX, S.XXX, S.WED, S.WDE, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX},
        /*L4*/      {S.WDE, S.WDE, S.XXX, S.XXX, S.WED, S.WDE, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX},
        /*NET*/     {S.WDE, S.WDE, S.XXX, S.XXX, S.UUU, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.WDE, S.XXX, S.XXX, S.XXX, S.XXX},
        /*PROC*/    {S.WDE, S.WDE, S.XXX, S.XXX, S.WED, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.OOO, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX},
        /*DR_EMP*/  {S.DWE, S.DWE, S.DWE, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX},
        /*DR_COR*/  {S.DWE, S.DWE, S.XXX, S.XXX, S.XXX, S.DWE, S.DWE, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX},
        /*DR_ALG*/  {S.UUU, S.UUU, S.XXX, S.XXX, S.UUU, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.UUU, S.XXX, S.XXX, S.XXX, S.XXX},
        /*OTHER*/   {S.OOO, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX}
    };

    public static S getSequence(State from, State to) {
        return matrix[from.ordinal()][to.ordinal()];    
    }
 
}

