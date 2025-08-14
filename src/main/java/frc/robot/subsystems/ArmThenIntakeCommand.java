package frc.robot.subsystems; // <-- change to frc.robot.commands if you keep commands in a separate folder

import edu.wpi.first.wpilibj2.command.Command;

public class ArmThenIntakeCommand extends Command {
    private final ArmMotorSubsystem armMotorSS;
    private final IntakeMotorSubsystem intakeMotorSS;
    private final double armTarget;
    private final double intakeTarget;
    private boolean intakeStarted = false;

    public ArmThenIntakeCommand(ArmMotorSubsystem armMotorSS, IntakeMotorSubsystem intakeMotorSS, double armTarget, double intakeTarget) {
        this.armMotorSS = armMotorSS;
        this.intakeMotorSS = intakeMotorSS;
        this.armTarget = armTarget;
        this.intakeTarget = intakeTarget;

        addRequirements(armMotorSS, intakeMotorSS);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        armMotorSS.driveMotorToPos(armTarget, ArmMotorSubsystem.kSlot);

        double currentArmPos = armMotorSS.armMotor.getEncoder().getPosition();

        if (!intakeStarted && Math.abs(currentArmPos - armTarget) < 0.1) {
            intakeMotorSS.driveMotorToPos(intakeTarget, IntakeMotorSubsystem.kSlot);
            intakeStarted = true;
        }
    }

    @Override
    public boolean isFinished() {
        return intakeStarted &&
               Math.abs(intakeMotorSS.intakeMotor.getEncoder().getPosition() - intakeTarget) < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
