// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.elbow_subsystem.ElbowElevationRotationCommand;
import frc.robot.subsystems.elbow_subsystem.ElbowSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToPosCommand;
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

        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        //elevator levels
        // test 1
        joystick.a().onTrue(new ElevatorToPosCommand(ElevatorSubsystem.LOW_POSITION, elevatorSubsystem));
        //joystick.a().onTrue(new ElevatorToPosCommand(ElevatorSubsystem.LEVEL1_POSITION, elevatorSubsystem));
        joystick.b().onTrue(new ElevatorToPosCommand(ElevatorSubsystem.LEVEL2_POSITION, elevatorSubsystem));
        joystick.x().onTrue(new ElevatorToPosCommand(ElevatorSubsystem.LEVEL3_POSITION, elevatorSubsystem));
        joystick.y().onTrue(new ElevatorToPosCommand(ElevatorSubsystem.LEVEL4_POSITION, elevatorSubsystem));

        //elbow controlr
        // test 3 with no rotation + teat 4 with rotation
        // joystick.povLeft().onTrue(new ElbowElevationRotationCommand(elbowSubsystem.HORIZONTAL_POS_ELEVATION, elbowSubsystem.HORIZONTAL_POS_ROTATION + elbowSubsystem.CORAL_COMPENSATION, elbowSubsystem));
        // joystick.povRight().onTrue(new ElbowElevationRotationCommand(elbowSubsystem.HORIZONTAL_POS_ELEVATION, elbowSubsystem.HORIZONTAL_POS_ROTATION - elbowSubsystem.CORAL_COMPENSATION, elbowSubsystem));
        // joystick.povLeft().and(joystick.povRight()).whileFalse(elbowSubsystem.elevation > elbowSubsystem.HORIZONTAL_POS_ELEVATION? new ElbowElevationRotationCommand(elbowSubsystem.elevation, elbowSubsystem.HORIZONTAL_POS_ROTATION, elbowSubsystem): new ElbowElevationRotationCommand(elbowSubsystem.elevation, elbowSubsystem.rotation, elbowSubsystem));
        
        // joystick.povUp().onTrue(new ElbowElevationRotationCommand(elbowSubsystem.START_POS_ELEVATION, elbowSubsystem.START_POS_ROTATION, elbowSubsystem));
        // joystick.povDown().onTrue(new ElbowElevationRotationCommand(elbowSubsystem.INTAKE_POS_ELEVATION, elbowSubsystem.INTAKE_POS_ROTATION, elbowSubsystem));

        //intake control 
        //test 2
        joystick.leftTrigger().onTrue(Commands.runOnce(() -> {intakeSubsystem.intake();}));
        joystick.rightTrigger().onTrue(Commands.runOnce(() -> {intakeSubsystem.outtake();}));
        joystick.leftTrigger().and(joystick.rightTrigger()).onFalse(Commands.runOnce(() -> {intakeSubsystem.stop();}));

    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            Commands.print("No autonomous command configured")
        );
    }
}