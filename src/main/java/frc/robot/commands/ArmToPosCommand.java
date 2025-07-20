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

public class ArmToPosCommand extends SequentialCommandGroup {
    // Safe Position Constants
    final double SAFE_ELEVATE_POS = -29.0;
    final double SAFE_ROTATE_POS = 0.0;


    /**
     * Constructor for automated arm movement command.
     * @param elevatorSubsystem - Object of Elevator Subsystem
     * @param elbowSubsystem - Object of Elbow Subsystem
     * @param elevatorPos - Elevator position to go to (Inch)
     * @param elbowTargetPos = Elbow position to go to (Degrees)
     */
    public ArmToPosCommand(ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem, double elevatorPos, 
            double[] elbowTargetPos, double curElbowElevationPos, double curElbowRotationPos, double curElevatorPos) {

        
        this.addRequirements(elevatorSubsystem, elbowSubsystem);
        // Safe Position Variables
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
        if (curElevatorPos < 1.0) {
            elevatorTooLow = true;
        }
        else {
            elevatorTooLow = false;
        }


        // Sequential sequence for arm movements
        Command armMovement = new SequentialCommandGroup(
            // If elevator too low, move it up
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new ElevatorToPosCommand(ElevatorSubsystem.LOW_POSITION, elevatorSubsystem),
                    Commands.waitSeconds(0.5)), 
                new InstantCommand(){}, 
                () -> elevatorTooLow),

            

            //// Move Arm to a safe position
            // If Elbow elevation is not safe, move it to a safe elevation. If it is safe, do nothing.
            new ConditionalCommand(
                new InstantCommand(){}, 
                new SequentialCommandGroup(
                    new ElbowElevationRotationCommand(SAFE_ELEVATE_POS, curElbowRotationPos, elbowSubsystem),
                    Commands.waitSeconds(0.5)), 
                () -> elbowElevationSafe
            ),

            // Check if Elbow was elevated
            new ConditionalCommand(
                // Elbow elevation was not moved, and Elbow rotation is in safe pos. Do nothing.
                new ConditionalCommand(
                    new InstantCommand(){}, 
                    new SequentialCommandGroup(
                        new ElbowElevationRotationCommand(curElbowRotationPos, SAFE_ROTATE_POS, elbowSubsystem),
                        Commands.waitSeconds(0.5)), 
                    () -> elbowRotationSafe
                ),
                // Elbow elevation was moved, and Elbow rotation is not in safe pos. Rotate elbow to safe pos.
                new ConditionalCommand(
                    new InstantCommand(){}, 
                    new SequentialCommandGroup(
                        new ElbowElevationRotationCommand(SAFE_ELEVATE_POS, SAFE_ROTATE_POS, elbowSubsystem),
                        Commands.waitSeconds(0.5)), 
                    () -> elbowRotationSafe
                ),
                () -> elbowElevationSafe),

            //// Elbow now in safe position, move Elbow first, then Elevator to target position
            // Move Elbow to target position
            new ConditionalCommand(
                new ElbowElevationRotationCommand(curElbowElevationPos, elbowTargetPos[1], elbowSubsystem), 
                new ElbowElevationRotationCommand(SAFE_ELEVATE_POS, elbowTargetPos[1], elbowSubsystem), 
                () -> elbowElevationSafe),
            new ElbowElevationRotationCommand(elbowTargetPos[0], elbowTargetPos[1], elbowSubsystem),

            Commands.waitSeconds(0.5),

            // Move Elevator to target position
            new ElevatorToPosCommand(elevatorPos, elevatorSubsystem)
        );

        // Run command
        addCommands(armMovement);
    }
}