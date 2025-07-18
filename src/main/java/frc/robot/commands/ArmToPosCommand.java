package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elbow_subsystem.ElbowElevationRotationCommand;
import frc.robot.subsystems.elbow_subsystem.ElbowSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToPosCommand;

public class ArmToPosCommand extends SequentialCommandGroup {


    public ArmToPosCommand(ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem, double elevatorLevel, double[] elbowTargetPos) {
        double curElbowRotationPos = elbowSubsystem.getElevationPos();
        double curElbowElevationPos = elbowSubsystem.getRotationPos();
        double curElevatorPos = elevatorSubsystem.getPosition();

        boolean elbowElevationSafe;
        boolean elbowRotationSafe;
        boolean elevatorAboveLow;

        final double SAFE_ELEVATE_POS = -29.0;
        final double SAFE_ROTATE_POS = 0.0;

        // // Check elevator position
        // if (curElevatorPos < ElevatorSubsystem.LOW_POSITION) {
        //     ele
        // }

        // Check elevation safety
        if (curElbowElevationPos < -30 || curElbowElevationPos > -26) {
            elbowElevationSafe = false;
        }
        else {
            elbowElevationSafe = true;
        }

        // Check rotation safety
        if (curElbowRotationPos != 0) {
            elbowRotationSafe = false;
        }
        else {
            elbowRotationSafe = true;
        }

        Command armMovement = new SequentialCommandGroup(
            // Move Elevator up to 

            //// Move Arm to a safe position
            // If Elbow elevation not safe, move to safe elevation. If it is, do nothing
            new ConditionalCommand(
                new ElbowElevationRotationCommand(curElbowElevationPos, curElbowRotationPos, elbowSubsystem), 
                new ElbowElevationRotationCommand(SAFE_ELEVATE_POS, curElbowRotationPos, elbowSubsystem), 
                () -> elbowElevationSafe
            ),

            // Check if Elbow was elevated
            new ConditionalCommand(
                // Elbow was not moved, and Elbow rotation is in safe pos. Do nothing.
                new ConditionalCommand(
                    new ElbowElevationRotationCommand(curElbowElevationPos, curElbowRotationPos, elbowSubsystem), 
                    new ElbowElevationRotationCommand(curElbowRotationPos, SAFE_ROTATE_POS, elbowSubsystem), 
                    () -> elbowRotationSafe
                ),
                // Elbow was moved, and Elbow rotation is not in safe pos. Rotate elbow to safe pos.
                new ConditionalCommand(
                    new ElbowElevationRotationCommand(SAFE_ELEVATE_POS, curElbowRotationPos, elbowSubsystem), 
                    new ElbowElevationRotationCommand(SAFE_ELEVATE_POS, SAFE_ROTATE_POS, elbowSubsystem), 
                    () -> elbowRotationSafe
                ),
                () -> elbowElevationSafe),

            //// Elbow now in safe position, move Elevator first, then Elbow to target position
            // Move Elevator to target position
            new ElevatorToPosCommand(elevatorLevel, elevatorSubsystem),

            // Move Elbow to target position
            // startElbowRotationPos = elbowSubsystem.getRotationPos(),
            new ElbowElevationRotationCommand(curElbowElevationPos, elbowTargetPos[1], elbowSubsystem), 
            new ElbowElevationRotationCommand(elbowTargetPos[0], elbowTargetPos[1], elbowSubsystem)
        );

        addCommands(armMovement);
    }
    @Override
    public void initialize() {
      
    }
}