// Original program written by Summit Robotics https://github.com/SummitRobotics/FRC2025

package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.scoring_subsystem.ScoringSubsystem;
import frc.robot.subsystems.scoring_subsystem.ScoringSubsystem.Position;
import frc.robot.subsystems.scoring_subsystem.elevator.ElevatorToPosCommand;

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
        one("1"),// the left one
        two("2");// the right 2
        public String name;
        private Side(String name) {
            this.name = name;
        }
    }

    public static class Node {
        public Position position;
        public Side side;
        public HexSide hexSide;
        public Node(Position position, HexSide hexSide, Side side) {
            this.position = position;
            this.side = side;
            this.hexSide = hexSide;
        }

        public String toString() {
            return "Hex: " + hexSide.name + ", Side: " + side.name + ", Lvl: " + position.toString();
        }
        
    }

    // Create the constraints to use while pathfinding
    private PathConstraints constraints = new PathConstraints(
            1.0, 4.0,
            Units.degreesToRadians(270), Units.degreesToRadians(360));

    /**
     * Constructor for the AutoPlace class, sets up the pathplanning details and create the commands for 
     * auto-scoring.
     * @param drivetrain - Instance of Drivetrain object
     * @param elevatorSubsystem - Instance of Elevator Subsystem object
     * @param elbowSubsystem - Instance of Elbow Subsystem object
     * @param node - Where to score
     */
    public AutoPlace(CommandSwerveDrivetrain drivetrain, ScoringSubsystem scoringSubsystem, Node node) {
        PathPlannerPath path;
        String pathName = "";
        // Name format is [side symbol][1/2] (e.g. A1, A2, B1, B2)
        pathName += node.hexSide.name;
        // If lvl 1, append "lvl1" to the path name. Otherwise, append the side name
        pathName += node.position == Position.SCORE_L1 ? "lvl1" : node.side.name;

        // Ensure the supplied path is valid
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            e.printStackTrace();
            throw (new RuntimeException("Loaded a path that does not exist."));
        }

        //// -------------------------------------------------------------------------------------------------
        //// ------------------------------------- Auto-placing sequence -------------------------------------
        //// -------------------------------------------------------------------------------------------------
        /* Step One:   Pathfind to starting point of path, then follow path to move infront of reef to score.
         *             (At the same time, move elevator to level one position. No higher as it may affect
         *              balance at high speed.)
         * Step Two:   Move elevator to correct level, and rotate elbow to the right angle and orientation.
         * Step Three: Drive forward to line up coral with reef.
         * Step Four:  Deposit coral to score.
         * Step Five:  Drive backwards, and lower elevator and coral elbow.
         */

        // Command to move the robot to the desired position along a path, running until path is complete
        Command stepOne_driveToPath = new ParallelDeadlineGroup(
            AutoBuilder.pathfindThenFollowPath(path, constraints), // On the fly pathfinding to the reef
            scoringSubsystem.moveArm(Position.DRIVE_WITH_CORAL) // Start moving the elevator to level one
        );

        // Command to move elevator to correct level, and rotate elbow to the right angle and orientation
        Command stepTwo_moveScoringSystems = new ParallelCommandGroup(
            scoringSubsystem.moveArm(node.position) // Move elevator to the correct level
            // Rotate Elbow for the right angle and orientation
        );

        // Drive forward for 1 second
        Command stepThree_driveForwardForScoring = new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityY(0.1))).repeatedly().withTimeout(.5) // Drive forward to score
            // Deposit coral
        );
        // Drive forward for 1 second
        Command stepfour_depositCoral = //new SequentialCommandGroup(
            new InstantCommand() // Drive forward to score
            // Deposit coral
            ;
        //);

        // Drive backwards, and lower elevator and coral elbow
        Command stepFive_driveBackwardsandLowerScoringSystem = new ParallelDeadlineGroup(
            new InstantCommand(() -> drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityY(-0.5))).repeatedly().withTimeout(1), // Drive backwards for one second
            scoringSubsystem.moveArm(Position.SAFETY) // TODO add more steps to actually score
            // Rotate Elbow for stowing
        );

        // If not a simulation
        if (!Utils.isSimulation()) {
            addCommands(stepOne_driveToPath,
                    stepTwo_moveScoringSystems,
                    stepThree_driveForwardForScoring,
                    stepfour_depositCoral,
                    stepFive_driveBackwardsandLowerScoringSystem
                    );

        // For in a simulation
        } else {
            // // Simplified place in simulation
            // Timer timer = new Timer();
            // addCommands(
            //     // Move the robot to desired position
            //     AutoBuilder.pathfindThenFollowPath(path, constraints),
            //      
            //     /*
            //      * No coral scoring in simulation
            //      */

            //     // Adjust target speed to accelerate backwards (-X in robot centric) for a period of time
            //     new InstantCommand(timer::restart),
            //     new InstantCommand(() -> drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-timer.get() * 4))).repeatedly().withDeadline(new WaitCommand(0.5))
            // );
        }
    }
}
