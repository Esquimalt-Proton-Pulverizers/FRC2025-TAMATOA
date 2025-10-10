package frc.robot.commands;

import javax.xml.crypto.dsig.Transform;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoScoringPathBuilder {
    public static PathPlannerPath A_L, A_R, B_L, B_R, C_L, C_R, D_L, D_R, E_L, E_R, F_L, F_R;
    private boolean debugMode = false;

    private static final double LEFT_POLE_OFFSET = - 0.2; //
    private static final double RIGHT_POLE_OFFSET =  0.2; //
    private static final double RADIAL_OFFSET =  1.26; //
    private static final double DEFAULT_DISTANCE =  0.5; //meters

    // All positions for Blue alliance, program auto flips for red alliance
    private static final Rotation2d RELATIVE_ROBOT_YAW = new Rotation2d( Units.degreesToRadians(90)); // A1 yaw of the robot when in scoring position
    private static final Rotation2d RELATIVE_PATH_HEADING = new Rotation2d(Units.degreesToRadians(180)); // A1 heading of the robot to the 
    private static final Rotation2d ROTATION_PER_SIDE = new Rotation2d(Units.degreesToRadians(60)); // the rotation per side of the reef to do transformations

    public static final PathConstraints SCORING_PATH_CONSTRAINTS = new PathConstraints(1, .5, 1, 1);
    public static final PathConstraints FIND_PATH_CONSTRAINTS = new PathConstraints(2, .2, 1, 1);



    private static final Translation2d REEF_CENTER = new Translation2d(4.5,4.0); //center of the blue reef in m field coordinates
    private static final Translation2d A_L_OFFSET = new Translation2d(RADIAL_OFFSET,LEFT_POLE_OFFSET);
    private static final Translation2d A_R_OFFSET = new Translation2d(RADIAL_OFFSET,RIGHT_POLE_OFFSET);

    //All the left Poses
    private static Pose2d robotPoseA_L = new Pose2d(REEF_CENTER.plus(A_L_OFFSET),RELATIVE_ROBOT_YAW);
    private static Pose2d robotPoseB_L = robotPoseA_L.rotateAround(REEF_CENTER, ROTATION_PER_SIDE.times(-1));
    private static Pose2d robotPoseC_L = robotPoseA_L.rotateAround(REEF_CENTER, ROTATION_PER_SIDE.times(-2));
    private static Pose2d robotPoseD_L = robotPoseA_L.rotateAround(REEF_CENTER, ROTATION_PER_SIDE.times(-3));
    private static Pose2d robotPoseE_L = robotPoseA_L.rotateAround(REEF_CENTER, ROTATION_PER_SIDE.times(-4));
    private static Pose2d robotPoseF_L = robotPoseA_L.rotateAround(REEF_CENTER, ROTATION_PER_SIDE.times(-5));

    //All the right Poses
    private static Pose2d robotPoseA_R = new Pose2d(REEF_CENTER.plus(A_R_OFFSET),RELATIVE_ROBOT_YAW);
    private static Pose2d robotPoseB_R = robotPoseA_R.rotateAround(REEF_CENTER, ROTATION_PER_SIDE.times(-1));
    private static Pose2d robotPoseC_R = robotPoseA_R.rotateAround(REEF_CENTER, ROTATION_PER_SIDE.times(-2));
    private static Pose2d robotPoseD_R = robotPoseA_R.rotateAround(REEF_CENTER, ROTATION_PER_SIDE.times(-3));
    private static Pose2d robotPoseE_R = robotPoseA_R.rotateAround(REEF_CENTER, ROTATION_PER_SIDE.times(-4));
    private static Pose2d robotPoseF_R = robotPoseA_R.rotateAround(REEF_CENTER, ROTATION_PER_SIDE.times(-5));

    public AutoScoringPathBuilder(boolean debugMode){
        this.debugMode = debugMode;
        A_L = generateScoringPath(robotPoseA_L, DEFAULT_DISTANCE);
        A_R = generateScoringPath(robotPoseA_R, DEFAULT_DISTANCE);
        B_L = generateScoringPath(robotPoseB_L, DEFAULT_DISTANCE);
        B_R = generateScoringPath(robotPoseB_R, DEFAULT_DISTANCE);
        C_L = generateScoringPath(robotPoseC_L, DEFAULT_DISTANCE);
        C_R = generateScoringPath(robotPoseC_R, DEFAULT_DISTANCE);
        D_L = generateScoringPath(robotPoseD_L, DEFAULT_DISTANCE);
        D_R = generateScoringPath(robotPoseD_R, DEFAULT_DISTANCE);
        E_L = generateScoringPath(robotPoseE_L, DEFAULT_DISTANCE);
        E_R = generateScoringPath(robotPoseE_R, DEFAULT_DISTANCE);
        F_L = generateScoringPath(robotPoseF_L, DEFAULT_DISTANCE);
        F_R = generateScoringPath(robotPoseF_R, DEFAULT_DISTANCE);
        
    }

    public PathPlannerPath generateScoringPath(Pose2d robotFinalPose, double distanceM){
        Rotation2d robotFinalHeading = robotFinalPose.getRotation().plus(RELATIVE_PATH_HEADING).minus(RELATIVE_ROBOT_YAW);
        return generateScoringPath(robotFinalPose,robotFinalHeading, distanceM);
    }

    public PathPlannerPath generateScoringPath(Pose2d robotFinalPose, Rotation2d robotFinalHeading, double distanceM){
        Translation2d robotFinalTranslation = robotFinalPose.getTranslation();
        Translation2d pathTranslation = new Translation2d(robotFinalHeading.getCos()*distanceM,robotFinalHeading.getSin()*distanceM);

        Pose2d startPoint = new Pose2d(robotFinalTranslation.minus(pathTranslation), robotFinalHeading);
        Pose2d endPoint= new Pose2d(robotFinalTranslation, robotFinalHeading);
        Rotation2d endRobotYaw = robotFinalPose.getRotation();//.times(-1);
        IdealStartingState idealStartingState = new IdealStartingState(0, endRobotYaw);
        GoalEndState goalEndState = new GoalEndState(0, endRobotYaw);
        if (debugMode){
            System.out.println("startPoint "+ startPoint);
            System.out.println("endPoint "+ endPoint);
            System.out.println("IdealStartingState orientation " + endRobotYaw);
            System.out.println("goalEndState orientation " + endRobotYaw);
        }

        return new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(startPoint , endPoint),
            SCORING_PATH_CONSTRAINTS,
            idealStartingState,
            goalEndState
            );
    }





    public Command generateScoringPathTest(){
        return new InstantCommand(()->{
            System.out.println("Path A_L");
            generateScoringPath(robotPoseA_L, DEFAULT_DISTANCE);
            System.out.println("Path B_L");
            generateScoringPath(robotPoseB_L, DEFAULT_DISTANCE);
            System.out.println("Path C_L");
            generateScoringPath(robotPoseC_L, DEFAULT_DISTANCE);
            System.out.println("Path D_L");
            generateScoringPath(robotPoseD_L, DEFAULT_DISTANCE);
            System.out.println("Path E_L");
            generateScoringPath(robotPoseE_L, DEFAULT_DISTANCE);
            System.out.println("Path F_L");
            generateScoringPath(robotPoseF_L, DEFAULT_DISTANCE);
            }
        );
    }

    public Command printPathwaypointsTest(PathPlannerPath path){
        return new InstantCommand(()->{
            System.out.println("Path waypoints****************");
            System.out.println(path.getWaypoints());
            }
        );
    }

    public Command goToScoringPosition(PathPlannerPath path){
        return AutoBuilder.pathfindThenFollowPath(path, FIND_PATH_CONSTRAINTS);
    }

}
