package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
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
     * 
     * @param elevatorSubsystem
     * @param elbowSubsystem
     * @param elevatorLevel
     * @param elbowTargetPos
     */
    public ArmToPosCommand(ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem, double elevatorLevel, 
            double[] elbowTargetPos, double curElbowElevationPos, double curElbowRotationPos, double curElevatorPos) {

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