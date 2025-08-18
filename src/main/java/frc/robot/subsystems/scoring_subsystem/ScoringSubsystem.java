package frc.robot.subsystems.scoring_subsystem;

import javax.naming.spi.StateFactory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elbow_subsystem.ElbowSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ScoringSubsystem {
    State currentState = State.CORAL_GROUND_INTAKE;
    State targetState = State.SCORE_L2;
    double[] targetVals;

    public Command moveArm(State currentState, State targetState, ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem){
        if (targetState == State.CORAL_GROUND_INTAKE)   {
                targetVals = new double[]{0.0, 0, 0};
        } else if (targetState == State.ALGAE_GROUND_INTAKE)   {
                targetVals = new double[]{0.0, 0, 0};
        }

        S seq = getSequence(currentState, targetState);
    
        return switch (seq) {
            case WDE -> returnWDECommand();
            case WED -> returnWEDCommand();
            case DWE -> returnDWECommand();
            case DEW -> returnDEWCommand();
            case EDW -> returnEDWCommand();
            case EWD -> returnEWDCommand();
            //case OOO -> returnOther();
            case XXX -> returnWDECommand();
            
            default -> throw new IllegalStateException("Unexpected sequence: " + seq);
        };
    }
    
    
    
    private Command returnWDECommand() {
        return new InstantCommand();
    }

    private Command returnWEDCommand() {
        return new InstantCommand();
    }

    private Command returnDWECommand() {
        return new InstantCommand();
    }

    private Command returnDEWCommand() {
        return new InstantCommand();
    }

    private Command returnEDWCommand() {
        return new InstantCommand();
    }

    private Command returnEWDCommand() {
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

