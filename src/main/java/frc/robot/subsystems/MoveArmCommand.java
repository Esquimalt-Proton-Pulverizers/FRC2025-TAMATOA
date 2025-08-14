// package frc.robot.subsystems;



// public class MoveArmCommand extends CommandBase{
//     private final ArmMotorSubsystem armMotor;
//     private final double target;

//     public MoveArmCommand(ArmMotorSubsystem armMotor, double target) {
//         this.armMotor = armMotor;
//         this.target = target;
//         addRequirements(armMotor);
//     }

//     @Override
//     public void initialize() {
//         armMotor.driveMotorToPos(target, ArmMotorSubsystem.kSlot);
//     }

//     @Override
//     public boolean isFinished() {
//         return Math.abs(armMotor.armMotor.getEncoder().getPosition() - target) < 0.1;
//     }
// }
