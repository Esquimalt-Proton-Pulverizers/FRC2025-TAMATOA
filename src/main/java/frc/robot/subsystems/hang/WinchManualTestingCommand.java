package frc.robot.subsystems.hang;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class WinchManualTestingCommand extends Command {
    private HangingSubsystem hangingSubsystem;
    private double targetPosition;
    private Timer timer = new Timer();

    public WinchManualTestingCommand(HangingSubsystem hangingSubsystem, double speed) {
        this.hangingSubsystem = hangingSubsystem;
        targetPosition = speed;
    }

    @Override
    public void initialize() {
        hangingSubsystem.setWinchPosition(targetPosition);
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(hangingSubsystem.getWinchPosition() - targetPosition) < 10.0)||timer.hasElapsed(5);
    }
}