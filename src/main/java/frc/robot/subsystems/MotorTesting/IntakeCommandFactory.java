package frc.robot.subsystems.MotorTesting;

// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkClosedLoopController;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import frc.robot.subsystems.intakeSubsystem.IntakeSubsystem;

// public class IntakeCommandFactory {
//     private final IntakeMotorSubsystem motorTestingSubsystem;
//     private double motorPos;

//     public IntakeCommandFactory(IntakeMotorSubsystem motorTestingSubsystem) {
//         this.motorTestingSubsystem = motorTestingSubsystem;
//         this.motorPos = 0; 
//     }

//     public static Command createIntakeToPosCommand(IntakeMotorSubsystem motorTestingSubsystem) {
//         return new RunCommand(
//             () -> motorTestingSubsystem.runMotorToPos(),
//             motorTestingSubsystem
//         );
//     }
// }
