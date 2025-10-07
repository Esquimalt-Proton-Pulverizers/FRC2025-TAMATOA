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

    public static Command followRelativePathCommand (Pose2d relativeMove, PathConstraints pathConstraints, CommandSwerveDrivetrain drivetrain) {
        System.out.println(" ****************************Following relative path step one: " + relativeMove);
        printPose(drivetrain);
        return Commands.defer(() -> {
            System.out.println("*********************Following relative path: " + relativeMove);
            Pose2d startPose = drivetrain.getState().Pose;
            Rotation2d startHeading = startPose.getRotation();
            Translation2d fieldTranslation = relativeMove.getTranslation().rotateBy(startHeading);
            Pose2d endPose = new Pose2d(startPose.getTranslation().plus(fieldTranslation), startHeading.plus(relativeMove.getRotation()));
            
            return AutoBuilder.pathfindToPose(endPose, pathConstraints);
        },
        Set.of(drivetrain));
              
    }
    public static void printPose(CommandSwerveDrivetrain drivetrain){
		Pose2d test = drivetrain.getState().Pose;
		System.out.println("x " + test.getX());
		System.out.println("y " + test.getY());
		System.out.println("rot " + test.getRotation());
	}
}


