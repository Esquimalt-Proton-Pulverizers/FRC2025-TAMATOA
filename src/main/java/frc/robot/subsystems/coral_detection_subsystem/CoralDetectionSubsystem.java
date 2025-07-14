package frc.robot.subsystems.coral_detection_subsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralDetectionSubsystem extends SubsystemBase {
    private static final int DEFAULT_PIPELINE = 2;
    private static final int AI_PIPELINE = 1;
    private static Timer timer = new Timer();
    public CoralDetectionSubsystem() {
        LimelightHelpers.setPipelineIndex("coral", DEFAULT_PIPELINE);
        timer.reset();
    }

    public static double findCoralPos() {
        LimelightHelpers.setPipelineIndex("coral", AI_PIPELINE);
        timer.start();
        // double txnc = Double.NaN;
        // if(timer.hasElapsed(0.5)) {
        //     txnc = LimelightHelpers.getTXNC("coral");  // Horizontal offset from principal pixel/point to target in degrees
        //     LimelightHelpers.setPipelineIndex("coral", DEFAULT_PIPELINE);
        // }
        double txnc = LimelightHelpers.getTXNC("coral");  // Horizontal offset from principal pixel/point to target in degrees

        return txnc;
    }
}
