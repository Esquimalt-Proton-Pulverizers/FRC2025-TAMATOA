// Original program written by Summit Robotics https://github.com/SummitRobotics/FRC2025

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerveDrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToPosCommand;

public class AutoPickup extends SequentialCommandGroup {

    public static enum CoralStationSide {
        NONE("None"),
        LEFT("LeftStation"),
        RIGHT("RightStation");
        public String pathName;
        CoralStationSide(String pathName) {
            this.pathName = pathName;
        }
    }

    public static CoralStationSide getCoralSide(Pose2d pose) {
        // 4 meters is the midline of the field
        if (!DriverStation.getAlliance().isPresent() || DriverStation.getAlliance().get() == Alliance.Blue) {
            return pose.getY() > 4 ? CoralStationSide.LEFT : CoralStationSide.RIGHT;
        } else {
            return pose.getY() < 4 ? CoralStationSide.LEFT : CoralStationSide.RIGHT;
        }
    }

    public AutoPickup(CommandSwerveDrivetrain drivetrain, ElevatorSubsystem elevatorSubsystem, Supplier<CoralStationSide> side, boolean level1) {
        this(drivetrain, elevatorSubsystem, side, level1, "");
    }

    public AutoPickup(CommandSwerveDrivetrain drivetrain, ElevatorSubsystem elevatorSubsystem, Supplier<CoralStationSide> side, boolean level1, String suppliedPathName) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(270), Units.degreesToRadians(360));
        // Load the station side paths from the path planner
        PathPlannerPath leftPath;
        PathPlannerPath rightPath;
        try {
            leftPath = PathPlannerPath.fromPathFile(CoralStationSide.LEFT.pathName);
            rightPath = PathPlannerPath.fromPathFile(CoralStationSide.RIGHT.pathName);
        } catch (Exception e) {
            e.printStackTrace();
            throw (new RuntimeException("Loaded a path that does not exist."));
        }
        // If given a supplied path name then load it
        PathPlannerPath suppliedPath;
        if (!suppliedPathName.isEmpty()) {
            try {
                suppliedPath = PathPlannerPath.fromPathFile(suppliedPathName);
            } catch (Exception e) {
                throw new RuntimeException();
            }
        } else {
            // Set to leftPath if no supplied path name is given, but otherwise unused
            suppliedPath = leftPath;
        }

        if (!Utils.isSimulation()) {
            addCommands(
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        // Perform the auto pickup sequence
                        new ParallelCommandGroup(
                            // Move the elevator to the appropriate position based on the level1 flag
                            new SequentialCommandGroup(
                                new ElevatorToPosCommand(level1 ? ElevatorSubsystem.level2Position : ElevatorSubsystem.lowPosition, elevatorSubsystem)
                            ),
                            // Drive to the station
                            new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                    // Follow the suppplied path if given, otherwise pathfind and then follow the left or right path
                                    new ConditionalCommand(
                                        new ConditionalCommand(
                                            AutoBuilder.pathfindThenFollowPath(leftPath, constraints),
                                            AutoBuilder.pathfindThenFollowPath(rightPath, constraints),
                                            () -> side.get() == CoralStationSide.LEFT
                                        ),
                                        AutoBuilder.followPath(suppliedPath),
                                        () -> suppliedPathName.isEmpty()
                                    ),
                                    new PrintCommand("Pathfinding to station").repeatedly()
                                ),
                                // Path may end with a target velocity, to drive robot into station, do so for a short time
                                new WaitCommand(0.5),
                                // Stop the robot from moving into station, the robot will move backwards after coral intake so OK if never reach this
                                new InstantCommand(() -> drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)))
                            ).withName("Pathfinding to station")
                        ).withDeadline(
                            // Wait 1 second after reaching the station to ensure the robot is aligned
                            new WaitCommand(1)
                        ),
                        new PrintCommand("Finished auto align"),
                        // Move back from the station, while centering the coral in receive, for a short period
                        new ParallelCommandGroup(
                            new ElevatorToPosCommand(ElevatorSubsystem.lowPosition, elevatorSubsystem),
                            new InstantCommand(() -> drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-2))).repeatedly()
                        ).withDeadline(new WaitCommand(0.25)),
                        new PrintCommand("Drove backwards")
                    ),
                    // Do nothing if given the flag
                    new InstantCommand(() -> {}),
                    () -> side.get() != CoralStationSide.NONE
                )
            );
        } else {
            // Simulation is simplified pickup
            addCommands(
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        // Drive to the station
                        new ConditionalCommand(
                            new ConditionalCommand(
                                AutoBuilder.pathfindThenFollowPath(leftPath, constraints),
                                AutoBuilder.pathfindThenFollowPath(rightPath, constraints),
                                () -> side.get() == CoralStationSide.LEFT
                            ),
                            AutoBuilder.followPath(suppliedPath),
                            () -> suppliedPath != null
                        ),
                        // Move the robot back with a constant velocity (-X in robot centric) for a period of time
                        new InstantCommand(() -> drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-2))).repeatedly().withDeadline(new WaitCommand(0.25))
                    ),
                    // Do nothing if given the flag
                    new InstantCommand(() -> {}),
                    () -> side.get() != CoralStationSide.NONE
                )
            );
        }
    }
}
