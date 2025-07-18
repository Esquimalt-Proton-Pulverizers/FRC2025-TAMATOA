// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.ArmToPosCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.elbow_subsystem.ElbowElevationRotationCommand;
import frc.robot.subsystems.elbow_subsystem.ElbowSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hang.HangingSubsystem;
import frc.robot.subsystems.intake_subsystem.IntakeSubsystem;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double MaxControlSpeed = 1;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

	public final HangingSubsystem hanger = new HangingSubsystem();

    public final ElbowSubsystem elbowSubsystem = new ElbowSubsystem();

    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxControlSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxControlSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Arm controls
        // joystick.leftBumper().onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LOW_POSITION, ElbowSubsystem.INTAKE_POS));
        joystick.a().onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LEVEL1_POSITION, ElbowSubsystem.LOW_POS));
        joystick.b().onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LEVEL2_POSITION, ElbowSubsystem.MIDS_POS));
        joystick.x().onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LEVEL3_POSITION, ElbowSubsystem.MIDS_POS));
        joystick.y().onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LEVEL4_POSITION, ElbowSubsystem.HIGH_POS));

        // // Elbow control
        // // test 3 with no rotation + teat 4 with rotation
        // joystick.povLeft().onTrue(new ElbowElevationRotationCommand(ElbowSubsystem.HORIZONTAL_POS_ELEVATION, ElbowSubsystem.HORIZONTAL_POS_ROTATION, elbowSubsystem));
        // joystick.povRight().onTrue(new ElbowElevationRotationCommand(ElbowSubsystem.HORIZONTAL_POS_ELEVATION, ElbowSubsystem.HORIZONTAL_POS_ROTATION, elbowSubsystem));
        // joystick.povLeft().and(joystick.povRight()).whileFalse(
        //     elbowSubsystem.elevation > ElbowSubsystem.HORIZONTAL_POS_ELEVATION?
        //      new ElbowElevationRotationCommand(elbowSubsystem.elevation, elbowSubsystem.HORIZONTAL_POS_ROTATION, elbowSubsystem):
        //      new ElbowElevationRotationCommand(elbowSubsystem.elevation, elbowSubsystem.rotation, elbowSubsystem)
        // );
        
        // joystick.povUp().onTrue(new ElbowElevationRotationCommand(ElbowSubsystem.START_POS_ELEVATION, ElbowSubsystem.START_POS_ROTATION, elbowSubsystem));
        // joystick.povDown().onTrue(new ElbowElevationRotationCommand(ElbowSubsystem.INTAKE_POS_ELEVATION, ElbowSubsystem.INTAKE_POS_ROTATION, elbowSubsystem));

        // //intake control 
        // //test 2 semi-success, just have to move on
        // joystick.leftTrigger().onTrue(Commands.runOnce(() -> {intakeSubsystem.intake();}));
        // joystick.rightTrigger().onTrue(Commands.runOnce(() -> {intakeSubsystem.outtake();}));
        // joystick.leftTrigger().and(joystick.rightTrigger()).onFalse(Commands.runOnce(() -> {intakeSubsystem.stop();}));//works but funny


		// Hanger Control
		joystick.povUp().onTrue(hanger.extend());
        joystick.povDown().onTrue(hanger.retract());

		joystick.leftBumper().onTrue(hanger.intake());
        joystick.leftBumper().onFalse(hanger.stop());
        joystick.back().onTrue(hanger.manualRetract());
        joystick.back().onFalse(hanger.resetWinch());
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            Commands.print("No autonomous command configured")
        );
    }
}