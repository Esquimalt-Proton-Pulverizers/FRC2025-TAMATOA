package frc.robot.subsystems.arm_subsystem;

import java.util.EnumSet;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elbow_subsystem.ElbowSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ArmSubsystem extends SubsystemBase{
    
    protected ElbowSubsystem elbowSubsystem = new ElbowSubsystem();
    protected ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    

    private static Timer timer = new Timer();
    private static boolean hasBeenInitialized = false;

    protected static double elevationTargetPos = 0;
    protected static double rollTargetPos = 0;
    protected static double elevatorTargetPos = ElevatorSubsystem.LOW_POSITION;

    private static boolean reachedPosition = false;

    
    private EnumSet<ElbowState> elbowStates = EnumSet.noneOf(ElbowState.class);
    private EnumSet<WristState> wristStates = EnumSet.noneOf(WristState.class);
    private EnumSet<ElevatorState> elevatorStates = EnumSet.noneOf(ElevatorState.class);


    public enum ElevatorState {
        UNKNOWN,      // Elevator is below 20"
        SAFE_TO_DRIVE,        // Elevator position is safe for driving/moving the robot
        SAFE_FOR_ELBOW_MOVE,  // Elevator is in a position where it's safe for elbow to move
        AT_GROUND_POSITION    // Elevator is at or near ground (e.g., for intaking)
    }
    
    
    public enum ElbowState {
        SAFE_TO_MOVE_ELEVATOR_ABOVE_30,  // Elbow is in a position where elevator can go above 20"
        SAFE_TO_GO_TO_GROUND,            // Elbow is safe to lower to ground
        SAFE_TO_DRIVE,                   // Elbow is in a safe driving position
        IN_SCORING_POSITION,             // Elbow is positioned for scoring
        LOCKED_FOR_SAFETY                // Elbow movement is restricted for safety
    }
    
    public enum WristState {
        SAFE_WITH_ELBOW_FOR_GROUND, // Wrist is in a position safe when elbow is above a certain angle
        SAFE_FOR_ELBOW_ABOVE_THRESHOLD, // Wrist is safe when elbow is below a certain angle
        PARALLEL_TO_GROUND,              // Wrist is parallel to ground
        PERPENDICULAR_TO_ARM,            // Wrist is aligned with arm for certain tasks
        STOWED_POSITION,                 // Wrist is in a stowed position
        UNSAFE_POSITION_WARNING          // Wrist is in an unsafe config and needs to be corrected
    }


    public ArmSubsystem() {
        timer.start();

        
    }
    public boolean reachedPosition(){
        return reachedPosition;
    }


    protected static void setArmTargetPos(double elevatorInches, double elevationAngle, double rollAngle) {
        elevationTargetPos = elevationAngle;
        rollTargetPos = rollAngle;
        elevationTargetPos =elevatorInches;
        reachedPosition = false;

    }

    

    public static void initialize(){
        if(!hasBeenInitialized) {
            ElbowSubsystem.initialize();
            ElbowSubsystem.initialize();
            
        }
    }

    @Override
    public void periodic() {
      // Put code here to be run every loop
      if(timer.hasElapsed(2.0)) {

        System.out.println("---------ARM--------------");
        timer.reset();
      }
    }


    public boolean atPosition(){
        return false;
    }

    public Command sampleArmCommand() {
        return Commands.runOnce(() -> {
            
        });    
    }
    public Command setArmTargetPositionCommand(double elevatorInches, double elevationAngle, double rollAngle) {
        return Commands.runOnce(() -> {setArmTargetPos(elevatorInches, elevationAngle, rollAngle);});    
    }

    
}
/*Logic

    // Safe Position Constants
    final double SAFE_ELEVATE_POS = -29.0;
    final double SAFE_ROTATE_POS = 0.0;

        boolean elbowElevationSafe;
        boolean elbowRotationSafe;
        boolean elevatorTooLow;

        // Check elevation safety
        if (curElbowElevationPos < -30 || curElbowElevationPos > -26) {
            elbowElevationSafe = false;
        }
        else {
            elbowElevationSafe = true;
        }

        // Check rotation safety
        if (Math.abs(curElbowRotationPos) < 5.0) {
            elbowRotationSafe = true;
        }
        else {
            elbowRotationSafe = false;
        }

        // Check Elevator position
        if (curElevatorPos < 1) {
            elevatorTooLow = true;
        }
        else {
            elevatorTooLow = false;
        }


        // Sequential sequence for arm movements
        Command armMovement = new SequentialCommandGroup(
            // If elevator too low, move it up
            new ConditionalCommand(
                new ElevatorToPosCommand(ElevatorSubsystem.LOW_POSITION, elevatorSubsystem), 
                new InstantCommand(){}, 
                () -> elevatorTooLow),

            //// Move Arm to a safe position
            // If Elbow elevation is not safe, move it to a safe elevation. If it is safe, do nothing.
            new ConditionalCommand(
                new InstantCommand(){}, 
                new ElbowElevationRotationCommand(SAFE_ELEVATE_POS, curElbowRotationPos, elbowSubsystem), 
                () -> elbowElevationSafe
            ),

            // Check if Elbow was elevated
            new ConditionalCommand(
                // Elbow elevation was not moved, and Elbow rotation is in safe pos. Do nothing.
                new ConditionalCommand(
                    new InstantCommand(){}, 
                    new ElbowElevationRotationCommand(curElbowRotationPos, SAFE_ROTATE_POS, elbowSubsystem), 
                    () -> elbowRotationSafe
                ),
                // Elbow elevation was moved, and Elbow rotation is not in safe pos. Rotate elbow to safe pos.
                new ConditionalCommand(
                    new InstantCommand(){}, 
                    new ElbowElevationRotationCommand(SAFE_ELEVATE_POS, SAFE_ROTATE_POS, elbowSubsystem), 
                    () -> elbowRotationSafe
                ),
                () -> elbowElevationSafe),

            //// Elbow now in safe position, move Elevator first, then Elbow to target position
            // Move Elevator to target position
            new ElevatorToPosCommand(elevatorLevel, elevatorSubsystem),

            // Move Elbow to target position
            new ConditionalCommand(
                new ElbowElevationRotationCommand(curElbowElevationPos, elbowTargetPos[1], elbowSubsystem), 
                new ElbowElevationRotationCommand(SAFE_ELEVATE_POS, elbowTargetPos[1], elbowSubsystem), 
                () -> elbowElevationSafe),
            new ElbowElevationRotationCommand(elbowTargetPos[0], elbowTargetPos[1], elbowSubsystem)
        );

        addCommands(armMovement);
    }
}
Input target value
cancel other tasks

 */