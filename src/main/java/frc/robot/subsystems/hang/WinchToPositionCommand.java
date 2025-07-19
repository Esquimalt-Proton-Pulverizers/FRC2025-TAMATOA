package frc.robot.subsystems.hang;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class WinchToPositionCommand extends Command {
    private HangingSubsystem hangingSubsystem;
    private double targetPosition;
    private Timer timer = new Timer();

    public WinchToPositionCommand(HangingSubsystem hangingSubsystem, HangingSubsystem.WinchPosition targetPosition) {
        this.hangingSubsystem = hangingSubsystem;
        this.targetPosition = targetPosition.value;
    }

    public WinchToPositionCommand(HangingSubsystem hangingSubsystem, double targetPosition) {
        this.hangingSubsystem = hangingSubsystem;
        this.targetPosition = targetPosition;
    }

    @Override
    public void initialize() {
        hangingSubsystem.setWinchPosition(targetPosition);
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(hangingSubsystem.getWinchPosition() - targetPosition) < 10.0 && timer.hasElapsed(0.5)) || timer.hasElapsed(5.0);
    }
}