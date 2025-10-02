package frc.robot.subsystems.scoring_subsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.scoring_subsystem.differential.DifferentialElevationRotationCommand;
import frc.robot.subsystems.scoring_subsystem.differential.DifferentialSubsystem;
import frc.robot.subsystems.scoring_subsystem.differential.SmartElbowElevationCommand;
import frc.robot.subsystems.scoring_subsystem.elevator.ElevatorSubsystem;
import frc.robot.subsystems.scoring_subsystem.elevator.ElevatorToPosCommand;


public class ScoringSubsystem extends SubsystemBase{
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
	private final DifferentialSubsystem differentialSubsystem = new DifferentialSubsystem();
    private static final double SAFE_ELEVATOR_HEIGHT = 5.0; // Minimum safe height for elevator to avoid collisions
    public static Position currentPosition = Position.UNDEFINED; // Initial state; NOTE robot must start in this position or collisions may occur
    private double[] targetVals; // 1; diff pos, 2; Wrist pos, 3; Elevator pos
    private double startDiffElevationAngle;
    private double startDiffWristAngle;
    private double startElevPos;
    private Timer timer = new Timer();
    private boolean debugMode = true;
    //Scoring System Position Constants
    /*from elevator class*/
    public static final double LOCK_POSITION   =  0.0;
    public static final double LOW_POSITION    =  3.0; // Correct Low position value 2.0 
    public static final double LEVEL1_POSITION =  5.0;
    public static final double LEVEL2_POSITION = 16.5;
    public static final double LEVEL3_POSITION = LEVEL2_POSITION + 16.0;
    public static final double LEVEL4_POSITION = 58.0;
    public static final double NET_POSITION    = 50;
    public static final double CORAL_STATION_POSITION = 19.0;
    /*from Differential class*/
    private final static double START_POS_ELEVATION = 0.0;
    private final static double START_POS_ROTATION  = 0.0;
    private final static double[] HOMING_POS = {START_POS_ELEVATION - 5.0, START_POS_ROTATION};
    private final static double[] INTAKE_POS = {-98.0, 90.0};
    private final static double[] LOW_POS    = {-26.0, 90.0};
    private final static double[] MIDS_POS   = {-51.5,  0.0};
    private final static double[] HIGH_POS   = {-51.5,  0.0};
    private final static double[] CORAL_POS  = {-26.0, 90.0};
    /**From this class*/
    public enum Position {
        //Note: "+" sign is only added to disable the VScode inlay hints and make it easier to read
        CORAL_GROUND_INTAKE     (-98.0,     +90.0,  +3.0),
        ALGAE_GROUND_INTAKE     (-115.0,    -90.0,  +5.2), // not yet tested
        CORAL_STATION_INTAKE    (+360.0,      +0.0,   +0.0), // not yet determined
        ALGAE_LOLLIPOP_INTAKE   (-115.0,    -90.0,  +11.25), // not yet tested
        HOME_FOR_CLIMB          (-5.0,      +0.0,   +2.0),
        SET_CORAL_POSITION_LEFT (+360.0,      +0.0,   +0.0), // not yet determined
        SET_CORAL_POSITION_RIGHT(+360.0,      +0.0,   +0.0), // not yet determined
        SCORE_L1                (-26.0,     +90.0,  +5.0),
        SCORE_L2                (-51.5,     +0.0,   +16.5),
        SCORE_L3                (-51.5,     +0.0,   +32.5),
        SCORE_L4                (-51.5,     +0.0,   +58.0),
        SCORE_NET               (+360.0,      +0.0,   +0.0), // not yet determined
        SCORE_PROCESSOR         (-90.0,     +180.0, +4.0), // not yet tested
        DRIVE_EMPTY             (-5.0,      +0.0,   +2.0),
        DRIVE_WITH_CORAL        (+20.0,     +90.0,  +2.0), // not yet tested
        DRIVE_WITH_ALGAE        (+30.0,     +180.0, +2.0), // not yet tested
        SAFETY                  (-45.0,     +0.0,   +5.0),
        UNDEFINED(Double.NaN, Double.NaN, Double.NaN); // sentinel 
    
        private final double[] targetVals;
    
        Position(double diffElevationDeg, double diffWristDeg, double elevatorInches) {
            this.targetVals = new double[]{diffElevationDeg, diffWristDeg, elevatorInches};
        }
    
        public double[] getTargetVals() {
            return targetVals;
        }
        /**
         * Find the first matching Position given current values. Within "Tolerances"
         * @param current current values [val1, val2, val3]
         * @return matching current Position or UNDEFINED
         */
        public static Position fromCurrentVals(double[] current) {
            double[] tolerances = {5.0, 5.0, 2.0}; //TODO set these tolerances appropriately
            double safetyBonustolerance = 5.0; //extra tolerance for safety position
            for (Position pos : values()) {
                if (pos == UNDEFINED) continue; // skip sentinel entry for undefined in a way that is ok if we add more positions later

                double[] target = pos.getTargetVals();
                boolean withinTolerance =
                        Math.abs(current[0] - target[0]) <= tolerances[0] &&
                        Math.abs(current[1] - target[1]) <= tolerances[1] &&
                        Math.abs(current[2] - target[2]) <= tolerances[2];
                if(pos == SAFETY){ 
                    withinTolerance =
                        Math.abs(current[0] - target[0]) <= tolerances[0] + safetyBonustolerance &&
                        Math.abs(current[1] - target[1]) <= tolerances[1] + safetyBonustolerance && 
                        current[2] > SAFE_ELEVATOR_HEIGHT; //elevator just needs to be above x inches TODO go over this logic
                }

                if (withinTolerance) {
                    return pos;
                }
            }
            return UNDEFINED;
        }
    }

    
    public void initialize() {
        differentialSubsystem.initialize(); //TODO make a class variable for this, or pull from start position
    }
    public ScoringSubsystem() {
        timer.start();
    }
    @Override
    public void periodic() {
        if(timer.hasElapsed(2.0)) {
            if (debugMode==true){
                double[] current = {differentialSubsystem.getElevationPos(), differentialSubsystem.getRotationPos(), elevatorSubsystem.getPosition()};
                System.out.println("Current ScoringSystem position" + Position.fromCurrentVals(current));
            }
            timer.reset();
         }
    }
    /**
     * Move the arm to the specified target position using an optimal sequence of movements.
     * @param targetPosition The desired target position.
     * @return A Command that executes the movement sequence.
     */
    public Command moveArm(Position targetPosition){
        //determine current position but save starting positions for movement commands
        startDiffElevationAngle = differentialSubsystem.getElevationPos();
        startDiffWristAngle = differentialSubsystem.getRotationPos();
        startElevPos = elevatorSubsystem.getPosition();
        double[] current = {startDiffElevationAngle, startDiffWristAngle, startElevPos}; 
        currentPosition = Position.fromCurrentVals(current);
        System.out.println("Starting position is: " + currentPosition);
        
        if (currentPosition == targetPosition) {
            System.out.println("moveArm not Skipped, but went to same position");
            /* System.out.println("moveArm Skipped");
            return new InstantCommand(); // No movement needed
            */
        }
        
        if (currentPosition == Position.UNDEFINED){
            System.out.println("Current position is undefined, moving to safe position first");
            // Move to a safe intermediate position before proceeding to the target
            double[] safeVals = {
                Position.SAFETY.getTargetVals()[0],
                Position.SAFETY.getTargetVals()[1], 
                startElevPos>SAFE_ELEVATOR_HEIGHT ? startElevPos : SAFE_ELEVATOR_HEIGHT + 2}; //keep current elev if above 5, otherwise go above it by a margin
                
            return returnDWECommand(safeVals, elevatorSubsystem,differentialSubsystem);
            //.andThen(moveArm(targetPosition)); //TODO, see if recursion works, if not, this should all be in a big sequential command
            //.andThen(returnDEWCommand(targetPosition.getTargetVals(), elevatorSubsystem, differentialSubsystem)); //TODO try this
        }
        targetVals = targetPosition.getTargetVals();
        
        S seq = getSequence(currentPosition, targetPosition);

        System.out.println("Switch 1: " + seq);
        return switch (seq) {
            case WDE -> returnWDECommand(targetVals, elevatorSubsystem, differentialSubsystem);
            case WED -> returnWEDCommand(targetVals, elevatorSubsystem, differentialSubsystem);
            case DWE -> returnDWECommand(targetVals, elevatorSubsystem, differentialSubsystem);
            case DEW -> returnDEWCommand(targetVals, elevatorSubsystem, differentialSubsystem);
            case EDW -> returnEDWCommand(targetVals, elevatorSubsystem, differentialSubsystem);
            case EWD -> returnEWDCommand(targetVals, elevatorSubsystem, differentialSubsystem);
            // case OOO -> returnOther();
            case x__ -> returnDWECommand(targetVals, elevatorSubsystem, differentialSubsystem); // order doesn't matter, so just pick one
            
            default -> throw new IllegalStateException("Unexpected sequence: " + seq);
        };
        
    }

    /** 
     * W is wrist, D is differential (sometimes called elbow), and E is elevator
     * is default, O is other, and U is still unknown and needs to be solved
     * they are three letters because the matrix is easier to read
     */
    public enum S {
        WDE, 
        WED, 
        DWE, 
        DEW, 
        EDW, 
        EWD,
        OOO, //other, this means it needs some special sequence that probably hasn't been made yet //TODO
        UUU, //unknown
        x__; //This means the order doesn't matter
    }

    // Matrix layout: rows = from state, cols = to state
    private static final S[][] matrix = {
        //           CG_IN  AG_IN  CS_IN  AL_LO  HOME   SET_L  SET_R  L1     L2     L3     L4     NET    PROC   DR_EMP DR_COR DR_ALG SAFETY
        /*CG_IN*/   {S.x__, S.EDW, S.x__, S.x__, S.WDE, S.OOO, S.OOO, S.x__, S.x__, S.x__, S.x__, S.WDE, S.x__, S.x__, S.x__, S.WDE, S.DWE}, //Coral Ground Intake
        /*AG_IN*/   {S.WDE, S.x__, S.x__, S.x__, S.WDE, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.OOO, S.WDE, S.x__, S.x__, S.x__, S.DWE}, //Algae Ground Intake
        /*CS_IN*/   {S.WDE, S.WDE, S.x__, S.WED, S.WDE, S.WDE, S.WDE, S.WED, S.x__, S.WED, S.WED, S.x__, S.x__, S.x__, S.x__, S.x__, S.DWE}, //Coral Station Intake
        /*AL_LO*/   {S.UUU, S.UUU, S.x__, S.x__, S.UUU, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.UUU, S.UUU, S.x__, S.x__, S.x__, S.DWE}, //Algae Lollipop Intake
        /*HOME*/    {S.DWE, S.DWE, S.DWE, S.UUU, S.x__, S.DWE, S.DWE, S.DWE, S.x__, S.x__, S.x__, S.DWE, S.DWE, S.x__, S.x__, S.DWE, S.DWE}, //Home for Climb
        /*SET_L*/   {S.UUU, S.UUU, S.x__, S.x__, S.WDE, S.x__, S.x__, S.WDE, S.x__, S.x__, S.EDW, S.x__, S.x__, S.x__, S.x__, S.x__, S.DWE}, //Set Coral Position Left
        /*SET_R*/   {S.UUU, S.UUU, S.x__, S.x__, S.OOO, S.x__, S.x__, S.WDE, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.DWE}, //Set Coral Position Right
        /*L1*/      {S.WDE, S.WDE, S.x__, S.x__, S.WED, S.WDE, S.WDE, S.x__, S.x__, S.WED, S.WED, S.x__, S.x__, S.x__, S.x__, S.x__, S.DWE}, //Level 1
        /*L2*/      {S.WDE, S.WDE, S.x__, S.x__, S.x__, S.WDE, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.DWE}, //Level 2
        /*L3*/      {S.WDE, S.WDE, S.x__, S.x__, S.WED, S.WDE, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.DWE}, //Level 3
        /*L4*/      {S.WDE, S.WDE, S.x__, S.x__, S.WED, S.WDE, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.DWE}, //Level 4
        /*NET*/     {S.WDE, S.WDE, S.x__, S.x__, S.UUU, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.WDE, S.x__, S.x__, S.x__, S.DWE}, //Net
        /*PROC*/    {S.WDE, S.WDE, S.x__, S.x__, S.WED, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.OOO, S.x__, S.x__, S.x__, S.x__, S.DWE}, //Processor
        /*DR_EMP*/  {S.DWE, S.DWE, S.DWE, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.DWE}, //Drive Empty
        /*DR_COR*/  {S.DWE, S.DWE, S.x__, S.x__, S.x__, S.DWE, S.DWE, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.DWE}, //Drive with Coral
        /*DR_ALG*/  {S.UUU, S.UUU, S.x__, S.x__, S.UUU, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.x__, S.UUU, S.x__, S.x__, S.x__, S.DWE}, //Drive with Algae
        /*SAFETY*/  {S.WDE, S.WED, S.UUU, S.WED, S.EWD, S.UUU, S.UUU, S.WDE, S.WDE, S.WDE, S.WDE, S.UUU, S.DWE, S.EWD, S.WED, S.WED, S.x__}  //Safety
        //Undefined state is not in the matrix because it is covered by an if statement in moveArm()
    };

    private S getSequence(Position from, Position to) {
        return matrix[from.ordinal()][to.ordinal()];    
    }

        
    private Command returnDWECommand(double[] targetVals, ElevatorSubsystem elevatorSubsystem, DifferentialSubsystem differentialSubsystem) {
        System.out.println("DWE run");
        return new SmartElbowElevationCommand(targetVals[0], differentialSubsystem, elevatorSubsystem)
        .andThen(new DifferentialElevationRotationCommand(targetVals[0], targetVals[1], differentialSubsystem))
        .andThen(new ElevatorToPosCommand(targetVals[2], elevatorSubsystem));
        
    }

    private Command returnDEWCommand(double[] targetVals, ElevatorSubsystem elevatorSubsystem, DifferentialSubsystem differentialSubsystem) {
        System.out.println("DEW run");
        return new SmartElbowElevationCommand(targetVals[0], differentialSubsystem, elevatorSubsystem)
        .andThen(new ElevatorToPosCommand(targetVals[2], elevatorSubsystem))
        .andThen(new DifferentialElevationRotationCommand(targetVals[0],targetVals[1], differentialSubsystem));
    }

    private Command returnWDECommand(double[] targetVals, ElevatorSubsystem elevatorSubsystem, DifferentialSubsystem differentialSubsystem) {
        System.out.println("WDE run");
        return new DifferentialElevationRotationCommand(startDiffElevationAngle, targetVals[1], differentialSubsystem)
        .andThen(new SmartElbowElevationCommand(targetVals[0], differentialSubsystem, elevatorSubsystem))
        .andThen(new ElevatorToPosCommand(targetVals[2], elevatorSubsystem));
    }

    private Command returnWEDCommand(double[] targetVals, ElevatorSubsystem elevatorSubsystem, DifferentialSubsystem differentialSubsystem) {
        System.out.println("WED run");
        return new DifferentialElevationRotationCommand(startDiffElevationAngle, targetVals[1], differentialSubsystem)
        .andThen(new ElevatorToPosCommand(targetVals[2], elevatorSubsystem))
        .andThen(new SmartElbowElevationCommand(targetVals[0], differentialSubsystem, elevatorSubsystem));
    }

    private Command returnEWDCommand(double[] targetVals, ElevatorSubsystem elevatorSubsystem, DifferentialSubsystem differentialSubsystem) {
        System.out.println("EWD run");
        return new ElevatorToPosCommand(targetVals[2], elevatorSubsystem)
        .andThen(new DifferentialElevationRotationCommand(startDiffElevationAngle, targetVals[1], differentialSubsystem))
        .andThen(new SmartElbowElevationCommand(targetVals[0], differentialSubsystem, elevatorSubsystem));
    }

    private Command returnEDWCommand(double[] targetVals, ElevatorSubsystem elevatorSubsystem, DifferentialSubsystem differentialSubsystem) {
        System.out.println("EDW run");
        return new ElevatorToPosCommand(targetVals[2], elevatorSubsystem)
        .andThen(new SmartElbowElevationCommand(targetVals[0], differentialSubsystem, elevatorSubsystem))
        .andThen(new DifferentialElevationRotationCommand(targetVals[0], targetVals[1], differentialSubsystem));
    }
   
}

