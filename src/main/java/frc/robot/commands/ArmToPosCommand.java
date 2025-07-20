package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elbow_subsystem.ElbowElevationRotationCommand;
import frc.robot.subsystems.elbow_subsystem.ElbowSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToPosCommand;

public class ArmToPosCommand extends Command {
    // Contructor Variables
    private ElevatorSubsystem elevatorSubsystem;
    private ElbowSubsystem elbowSubsystem;
    private double elevatorPos;
    private double[] elbowTargetPos;

    // Current Position Variables
    private double curElbowElevationPos;
    private double curElbowRotationPos;
    private double curElevatorPos;

    // Safe Position Variables
    private boolean elbowElevationSafe;
    private boolean elbowRotationSafe;
    private boolean elevatorTooLow;

    // Safe Position Constants
    private final double SAFE_ELEVATE_POS = -29.0;
    private final double SAFE_ROTATE_POS = 0.0;

    // Other Variables
    private boolean atPosition = false;


    /**
     * Constructor for automated arm movement command.
     * @param elevatorSubsystem - Object of Elevator Subsystem
     * @param elbowSubsystem - Object of Elbow Subsystem
     * @param elevatorPos - Elevator position to go to (Inch)
     * @param elbowTargetPos = Elbow position to go to (Degrees)
     */
    public ArmToPosCommand(ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem, 
            double elevatorPos, double[] elbowTargetPos) {
        
        // Saving the passed variables from constructor locally
        this.elevatorSubsystem = elevatorSubsystem;
        this.elbowSubsystem = elbowSubsystem;
        this.elevatorPos = elevatorPos;
        this.elbowTargetPos = elbowTargetPos;
        
        // Ensure there isn't two commands scheduled at the same time for the below subsystems
        this.addRequirements(elevatorSubsystem, elbowSubsystem);
    }

    private Command scheduledCommand;

    @Override
    public void initialize() {
        System.out.println("Starting ArmToPosCommand");
    
        curElbowElevationPos = elbowSubsystem.getElevationPos();
        curElbowRotationPos = elbowSubsystem.getRotationPos();
        curElevatorPos = elevatorSubsystem.getPosition();
    
        elbowElevationSafe = curElbowElevationPos >= -30 && curElbowElevationPos <= -26;
        elbowRotationSafe = Math.abs(curElbowRotationPos) < 5.0;
        elevatorTooLow = curElevatorPos < 1.0;
    
        atPosition = false;
    
        scheduledCommand = new SequentialCommandGroup(
            // If elevator too low, move it up
            new ConditionalCommand(
                new ElevatorToPosCommand(ElevatorSubsystem.LOW_POSITION, elevatorSubsystem),
                // new SequentialCommandGroup(
                //     new ElevatorToPosCommand(ElevatorSubsystem.LOW_POSITION, elevatorSubsystem),
                //     Commands.waitSeconds(0.5)), 
                new InstantCommand(), 
                () -> elevatorTooLow),
    
            // Move elbow to safe elevation
            new ConditionalCommand(
                new InstantCommand(), 
                new ElbowElevationRotationCommand(SAFE_ELEVATE_POS, curElbowRotationPos, elbowSubsystem),
                // new SequentialCommandGroup(
                //     new ElbowElevationRotationCommand(SAFE_ELEVATE_POS, curElbowRotationPos, elbowSubsystem),
                //     Commands.waitSeconds(0.5)), 
                () -> elbowElevationSafe),
    
            // If elevation moved, rotate if needed
            new ConditionalCommand(
                new ConditionalCommand(
                    new InstantCommand(),
                    new ElbowElevationRotationCommand(curElbowRotationPos, SAFE_ROTATE_POS, elbowSubsystem),
                    // new SequentialCommandGroup(
                    //     new ElbowElevationRotationCommand(curElbowRotationPos, SAFE_ROTATE_POS, elbowSubsystem),
                    //     Commands.waitSeconds(0.5)), 
                    () -> elbowRotationSafe),
                new ConditionalCommand(
                    new InstantCommand(),
                    new ElbowElevationRotationCommand(SAFE_ELEVATE_POS, SAFE_ROTATE_POS, elbowSubsystem),
                    // new SequentialCommandGroup(
                    //     new ElbowElevationRotationCommand(SAFE_ELEVATE_POS, SAFE_ROTATE_POS, elbowSubsystem),
                    //     Commands.waitSeconds(0.5)),
                    () -> elbowRotationSafe),
                () -> elbowElevationSafe),
    
            // Move elbow to target
            new ConditionalCommand(
                new ElbowElevationRotationCommand(curElbowElevationPos, elbowTargetPos[1], elbowSubsystem), 
                new ElbowElevationRotationCommand(SAFE_ELEVATE_POS, elbowTargetPos[1], elbowSubsystem), 
                () -> elbowElevationSafe),
            new ElbowElevationRotationCommand(elbowTargetPos[0], elbowTargetPos[1], elbowSubsystem),
    
            // Commands.waitSeconds(0.5),
    
            // Move elevator to target
            new ElevatorToPosCommand(elevatorPos, elevatorSubsystem),
    
            new InstantCommand(() -> atPosition = true)
        );
    
        // Schedule the command group
        scheduledCommand.schedule();
    }
    
    @Override
    public void execute() {
        // nothing needed here
    }
    
    @Override
    public void end(boolean interrupted) {
        if (scheduledCommand != null && scheduledCommand.isScheduled()) {
            scheduledCommand.cancel();
        }
    }
    
    @Override
    public boolean isFinished() {
        return atPosition;
    }
}