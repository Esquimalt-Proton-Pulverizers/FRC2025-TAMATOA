package frc.robot.subsystems.scoring_subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elbow_subsystem.ElbowElevationRotationCommand;
import frc.robot.subsystems.elbow_subsystem.ElbowSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToPosCommand;


public class ScoringSubsystem {
    public static State currentState = State.HOME_FOR_CLIMB; // Initial state; NOTE robot must start in this position or collisions may occur
    private double[] targetVals; // 1; diff pos, 2; Wrist pos, 3; Elevator pos
    private double elbElevation;
    private double elbRotation;
    private double elevation;
    private ElevatorSubsystem elevatorSubsystem;

    public Command moveArm(State targetState, ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem){
        this.elevatorSubsystem = elevatorSubsystem;
        currentState = getCurState(targetState);
        System.out.println("moveArmCalled");
        if (currentState == targetState) {
            return new InstantCommand(); // No movement needed
        }
        switch (targetState) {
            case CORAL_GROUND_INTAKE:
                targetVals = new double[]{-98.0, 90.0, 3.0}; 
                break;
            case ALGAE_GROUND_INTAKE:
                targetVals = new double[]{-115.0, -90.0, 5.2}; // not yet tested
                break;
            case CORAL_STATION_INTAKE: 
                targetVals = new double[]{0.0, 0, 0}; // not yet determined
                break;
            case ALGAE_LOLLIPOP_INTAKE:
                targetVals = new double[]{-115.0, -90.0, 11.25}; // not yet tested
                break;
            case HOME_FOR_CLIMB:
                targetVals = new double[]{-5.0, 0, 2}; 
                break;
            case SET_CORAL_POSITION_LEFT:
                targetVals = new double[]{0.0, 0, 0}; // not yet determined
                break;
            case SET_CORAL_POSITION_RIGHT:
                targetVals = new double[]{0.0, 0, 0}; // not yet determined
                break;
            case SCORE_L1:
                targetVals = new double[]{-26.0, 90.0, 5.0}; 
                break;
            case SCORE_L2: 
                targetVals = new double[]{-51.5, 0, 16.5}; 
                break;
            case SCORE_L3:
                targetVals = new double[]{-51.5, 0, 32.5}; 
                break;
            case SCORE_L4:  
                targetVals = new double[]{-51.5, 0.0, 58.0}; 
                break;
            case SCORE_NET:
                targetVals = new double[]{0.0, 0, 0}; // not yet determined
                break;
            case SCORE_PROCESSOR:
                targetVals = new double[]{-90, 180, 4.0}; // not yet tested
                break;
            case DRIVE_EMPTY:
                targetVals = new double[]{-5.0, 0, 2.0}; 
                break;
            case DRIVE_WITH_CORAL:
                targetVals = new double[]{20, 90, 2.0}; // not yet tested
                break;
            case DRIVE_WITH_ALGAE:
                targetVals = new double[]{30.0, 180, 2.0}; // not yet tested
                break;
            case OTHER:
                targetVals = new double[]{-45, 0, 5}; // not yet determined
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + targetState);
        }
        S seq = getSequence(currentState, targetState);

        System.out.println("Switch 1");
        return switch (seq) {
            case WDE -> returnWDECommand(targetVals, elevatorSubsystem, elbowSubsystem);
            case WED -> returnWEDCommand(targetVals, elevatorSubsystem, elbowSubsystem);
            case DWE -> returnDWECommand(targetVals, elevatorSubsystem, elbowSubsystem);
            case DEW -> returnDEWCommand(targetVals, elevatorSubsystem, elbowSubsystem);
            case EDW -> returnEDWCommand(targetVals, elevatorSubsystem, elbowSubsystem);
            case EWD -> returnEWDCommand(targetVals, elevatorSubsystem, elbowSubsystem);
            // case OOO -> returnOther();
            //case XXX -> returnInstantCommand();
            
            default -> throw new IllegalStateException("Unexpected sequence: " + seq);
        };
        
    }
    
    
    
    private Command returnInstantCommand() {
        return new InstantCommand();
    }
    public Command returnDWECommand(double[] targetVals, ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem) {
        System.out.println("DWE run");
        return new ElbowElevationRotationCommand(targetVals[0], elbRotation, elbowSubsystem)
        .andThen(new ElbowElevationRotationCommand(targetVals[0], targetVals[1], elbowSubsystem))
        .andThen(new ElevatorToPosCommand(targetVals[2], elevatorSubsystem));
        
    }

    private Command returnDEWCommand(double[] targetVals, ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem) {
        System.out.println("DEW run");
        return new ElbowElevationRotationCommand(targetVals[0], elbRotation, elbowSubsystem)
        .andThan(new ElevatorToPosCommand(targetVals[2], elevatorSubsystem))
        .andThan(new ElbowElevationRotationCommand(targetVals[0],targetVals[1], elbowSubsystem));
    }

    private Command returnWDECommand(double[] targetVals, ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem) {
        new ElbowElevationRotationCommand(elbElevation, targetVals[1], elbowSubsystem);
        new ElbowElevationRotationCommand(targetVals[0], targetVals[1], elbowSubsystem);
        new ElevatorToPosCommand(targetVals[2], elevatorSubsystem);
        return new InstantCommand();
    }

    private Command returnWEDCommand(double[] targetVals, ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem) {
        new ElbowElevationRotationCommand(elbElevation, targetVals[1], elbowSubsystem);
        new ElevatorToPosCommand(targetVals[2], elevatorSubsystem);
        new ElbowElevationRotationCommand(targetVals[0], targetVals[1], elbowSubsystem);
        return new InstantCommand();
    }

    private Command returnEWDCommand(double[] targetVals, ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem) {
        new ElevatorToPosCommand(targetVals[2], elevatorSubsystem);
        new ElbowElevationRotationCommand(elbElevation, targetVals[1], elbowSubsystem);
        new ElbowElevationRotationCommand(targetVals[0], targetVals[1], elbowSubsystem);
        return new InstantCommand();
    }

    private Command returnEDWCommand(double[] targetVals, ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem) {
        new ElevatorToPosCommand(targetVals[2], elevatorSubsystem);
        new ElbowElevationRotationCommand(targetVals[0], elbRotation, elbowSubsystem);
        new ElbowElevationRotationCommand(targetVals[0], targetVals[1], elbowSubsystem);
        return new InstantCommand();
    }






    public enum State {
        CORAL_GROUND_INTAKE,
        ALGAE_GROUND_INTAKE,
        CORAL_STATION_INTAKE,
        ALGAE_LOLLIPOP_INTAKE,
        HOME_FOR_CLIMB,
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
        OTHER, // Safe position for unknown states
        UNKNOWN // Used when current state cannot be determined, not any one actual position
    }

    /** 
     * W is wrist, D is differential (sometimes called elbow), and E is elevator
     * is default, O is other, and U is still unknown and needs to be solved
     * they are three letters because the matrix is easier to read
     */
    public enum S {
        WDE("1"), WED("2"), DWE("3"), DEW("4"), EDW("5"), EWD("6"),
        OOO("O"), UUU("U"), XXX("x");

        private final String label;
        S(String l) { this.label = l; }
        @Override 
        public String toString() { return label; }
    }

    // Matrix layout: rows = from state, cols = to state
    private static final S[][] matrix = {
        //           CG_IN  AG_IN  CS_IN  AL_LO  HOME   SET_L  SET_R  L1     L2     L3     L4     NET    PROC   DR_EMP DR_COR DR_ALG OTHER
        /*CG_IN*/   {S.XXX, S.EDW, S.XXX, S.XXX, S.WDE, S.OOO, S.OOO, S.XXX, S.XXX, S.XXX, S.XXX, S.WDE, S.XXX, S.XXX, S.XXX, S.WDE, S.DWE},
        /*AG_IN*/   {S.WDE, S.XXX, S.XXX, S.XXX, S.WDE, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.OOO, S.WDE, S.XXX, S.XXX, S.XXX, S.DWE},
        /*CS_IN*/   {S.WDE, S.WDE, S.XXX, S.WED, S.WDE, S.WDE, S.WDE, S.WED, S.XXX, S.WED, S.WED, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.DWE},
        /*AL_LO*/   {S.UUU, S.UUU, S.XXX, S.UUU, S.UUU, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.UUU, S.UUU, S.XXX, S.XXX, S.XXX, S.DWE},
        /*HOME*/    {S.DWE, S.DWE, S.DWE, S.UUU, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.DWE},
        /*SET_L*/   {S.UUU, S.UUU, S.XXX, S.XXX, S.WDE, S.OOO, S.XXX, S.WDE, S.XXX, S.XXX, S.EDW, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.DWE},
        /*SET_R*/   {S.UUU, S.UUU, S.XXX, S.XXX, S.OOO, S.XXX, S.OOO, S.WDE, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.DWE},
        /*L1*/      {S.WDE, S.WDE, S.XXX, S.XXX, S.WED, S.WDE, S.WDE, S.XXX, S.XXX, S.WED, S.WED, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.DWE},
        /*L2*/      {S.WDE, S.WDE, S.XXX, S.XXX, S.XXX, S.WDE, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.DWE},
        /*L3*/      {S.WDE, S.WDE, S.XXX, S.XXX, S.WED, S.WDE, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.DWE},
        /*L4*/      {S.WDE, S.WDE, S.XXX, S.XXX, S.WED, S.WDE, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.DWE},
        /*NET*/     {S.WDE, S.WDE, S.XXX, S.XXX, S.UUU, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.WDE, S.XXX, S.XXX, S.XXX, S.DWE},
        /*PROC*/    {S.WDE, S.WDE, S.XXX, S.XXX, S.WED, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.OOO, S.XXX, S.XXX, S.XXX, S.XXX, S.DWE},
        /*DR_EMP*/  {S.DWE, S.DWE, S.DWE, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.DWE},
        /*DR_COR*/  {S.DWE, S.DWE, S.XXX, S.XXX, S.XXX, S.DWE, S.DWE, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.DWE},
        /*DR_ALG*/  {S.UUU, S.UUU, S.XXX, S.XXX, S.UUU, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.UUU, S.XXX, S.XXX, S.XXX, S.DWE},
        /*OTHER*/   {S.OOO, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX},
        /*UNKNOWN*/ {S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.XXX, S.DWE}
    };

    public static S getSequence(State from, State to) {
        return matrix[from.ordinal()][to.ordinal()];    
    }
    private State getCurState(State targetState) {
        double leftMotorPos = ElbowSubsystem.leftElbowMotor.getEncoder().getPosition();
        double rightMotorPos = ElbowSubsystem.rightElbowMotor.getEncoder().getPosition();
        elbElevation = (rightMotorPos - leftMotorPos) / 2.0;
        elbRotation = (rightMotorPos + leftMotorPos) / 2.0;
        double elevatorPos = elevatorSubsystem.getPosition();
        State newCurState = State.UNKNOWN; // Default to target state if no match found (no Motion will ensue)
        if (Math.abs(elbElevation - (-98.0)) < 5.0 && Math.abs(elbRotation - 90.0) < 10.0 && Math.abs(elevatorPos - 3.0) < 2.0) {
            newCurState = State.CORAL_GROUND_INTAKE;
        } else if (Math.abs(elbElevation - (-115.0)) < 5.0 && Math.abs(elbRotation - (-90.0)) < 10.0 && Math.abs(elevatorPos - 5.2) < 1.0) {
            newCurState = State.ALGAE_GROUND_INTAKE;
        } else if (Math.abs(elbElevation - (0.0)) < 5.0 && Math.abs(elbRotation - 0.0) < 10.0 && Math.abs(elevatorPos - 0.0) < 1.0) {
            newCurState = State.CORAL_STATION_INTAKE;
        } else if (Math.abs(elbElevation - (-115.0)) < 5.0 && Math.abs(elbRotation - (-90.0)) < 10.0 && Math.abs(elevatorPos - 11.25) < 1.0) {
            newCurState = State.ALGAE_LOLLIPOP_INTAKE;
        } else if (Math.abs(elbElevation - (-5.0)) < 5.0 && Math.abs(elbRotation - 0.0) < 10.0 && Math.abs(elevatorPos - 0.0) < 1.0) {
            newCurState = State.HOME_FOR_CLIMB;
        } else if (Math.abs(elbElevation - (20.0)) < 5.0 && Math.abs(elbRotation - 90.0) < 10.0 && Math.abs(elevatorPos - 2.0) < 1.0) {
            newCurState = State.DRIVE_WITH_CORAL;
        } else if (Math.abs(elbElevation - (30.0)) < 5.0 && Math.abs(elbRotation - 180.0) < 10.0 && Math.abs(elevatorPos - 2.0) < 1.0) {
            newCurState = State.DRIVE_WITH_ALGAE;
        } else targetState = State.OTHER;
        return newCurState;
    }
 
}

