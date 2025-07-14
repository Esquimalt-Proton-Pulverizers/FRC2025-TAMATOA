// Original program written by Summit Robotics https://github.com/SummitRobotics/FRC2025

package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.elbow_subsystem.ElbowSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToPosCommand;

public class AutoPlace extends SequentialCommandGroup {

    public enum HexSide {
        A("A"),
        B("B"),
        C("C"),
        D("D"),
        E("E"),
        F("F");

        public String name;
        private HexSide(String name) {
            this.name = name;
        }
    }

    public enum Side {
        one("1"),
        two("2");
        public String name;
        private Side(String name) {
            this.name = name;
        }
    }

    public static class Node {
        public int level;
        public Side side;
        public HexSide hexSide;
        public Node(int level, HexSide hexSide, Side side) {
            this.level = level;
            this.side = side;
            this.hexSide = hexSide;
        }

        public String toString() {
            return "Hex: " + hexSide.name + ", Side: " + side.name + ", Lvl: " + level;
        }
    }

    // Create the constraints to use while pathfinding
    private PathConstraints constraints = new PathConstraints(
            1.0, 4.0,
            Units.degreesToRadians(270), Units.degreesToRadians(360));

    public AutoPlace(CommandSwerveDrivetrain drivetrain, ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem, Node node) {
        this(drivetrain, elevatorSubsystem, elbowSubsystem, node, "");
    }

    public AutoPlace(CommandSwerveDrivetrain drivetrain, ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem, Node node, String suppliedPathName) {
        PathPlannerPath path;
        String pathName = "";
        // Name format is [side symbol][1/2] (e.g. A1, A2, B1, B2)
        pathName += node.hexSide.name;
        // If lvl 1, append "lvl1" to the path name. Otherwise, append the side name
        pathName += String.valueOf(node.level).equals("1") ? "lvl1" : node.side.name;

        System.out.println("Pathname is: " + pathName);

        try {
            path = PathPlannerPath.fromPathFile(suppliedPathName.isEmpty() ? pathName : suppliedPathName);
        } catch (Exception e) {
            e.printStackTrace();
            throw (new RuntimeException("Loaded a path that does not exist."));
        }

        System.out.println("Before move command");


        // Command to move the robot to the desired position along a path, running until path is complete
        Command move = new ParallelDeadlineGroup(
            // On the fly pathfinding to the reef, or follow the path if supplied
            new ConditionalCommand(
                AutoBuilder.pathfindThenFollowPath(path, constraints),
                AutoBuilder.followPath(path),
                () -> suppliedPathName.isEmpty()
            
            ),
            // Start moving the elevator to the correct position
            //elevatorToLevel(node.level, elevatorSubsystem)
            new InstantCommand(()->System.out.println("inside move command") )

        );

        System.out.println("Exited move command");


        // // Command to move elevator to the desired position and score the coral
        // SequentialCommandGroup place = new SequentialCommandGroup(
        //     // Move elevator with a timeout
        //     elevatorToLevel(node.level, elevatorSubsystem)
        //     .withTimeout(1),
        //     // Wait time dependent on level
        //     new WaitCommand((node.level == 3) || (node.level == 4) ? 1 : 0.5)
            
        //     // Score Coral
        //     // new ConditionalCommand(
        //     //     new ParallelDeadlineGroup(
        //     //         // Add command to drive right
        //     //         new InstantCommand(() -> drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-0.5))).repeatedly().withTimeout(1)
        //     //     ),
        //     //     new InstantCommand(() -> new CoralDoorToPositionCommand(CoralDoorSubsystem.DoorPosition.OPEN, coralDoorSubsystem)),
        //     //     () -> node.level != 1
        //     // )
        // );

    //     if (!Utils.isSimulation()) {
    //         // Move the robot to desired position, stowing scrubber along the way
    //         addCommands(new ParallelDeadlineGroup(move));
    //         addCommands(place);

    //         // Lower elevator
    //         elevatorToLevel(0, elevatorSubsystem);

    //     } else {
            // Simplified place in simulation
            Timer timer = new Timer();
            addCommands(
                // Move the robot to desired position, stowing scrubber along the way
                new ConditionalCommand(
                    AutoBuilder.pathfindThenFollowPath(path, constraints),
                    AutoBuilder.followPath(path),
                    () -> suppliedPathName.isEmpty()
                ),

                /*
                 * No coral scoring in simulation
                 */

                // Adjust target speed to accelerate backwards (-X in robot centric) for a period of time
                new InstantCommand(timer::restart),
                new InstantCommand(() -> drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-timer.get() * 4))).repeatedly().withDeadline(new WaitCommand(0.5))
            );
    //     }
    }

    public Command elevatorToLevel(int level, ElevatorSubsystem elevatorSubsystem) {
        if (level == 1) {
            return new ElevatorToPosCommand(ElevatorSubsystem.LEVEL1_POSITION, elevatorSubsystem);
        }
        else if (level == 2) {
            return new ElevatorToPosCommand(ElevatorSubsystem.LEVEL2_POSITION, elevatorSubsystem);
        }
        else if (level == 3) {
            return new ElevatorToPosCommand(ElevatorSubsystem.LEVEL3_POSITION, elevatorSubsystem);
        }
        else if (level == 4) {
            return new ElevatorToPosCommand(ElevatorSubsystem.LEVEL4_POSITION, elevatorSubsystem);
        }
        else {
            return new ElevatorToPosCommand(ElevatorSubsystem.LOW_POSITION, elevatorSubsystem);
        }
    }
}
