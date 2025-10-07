package frc.robot.commands;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PathFinderHelperCommands {

    public Command followRelativePathCommand (Pose2d relativeMove, PathConstraints pathConstraints, CommandSwerveDrivetrain drivetrain) {
        return Commands.defer(() -> {
            Pose2d startPose = drivetrain.getState().Pose;
            Rotation2d startHeading = startPose.getRotation();
            Translation2d fieldTranslation = relativeMove.getTranslation().rotateBy(startHeading);
            Pose2d endPose = new Pose2d(startPose.getTranslation().plus(fieldTranslation), startHeading.plus(relativeMove.getRotation()));
            
            return AutoBuilder.pathfindToPose(endPose, pathConstraints);
        },
        Set.of(drivetrain));
              
    }
}


