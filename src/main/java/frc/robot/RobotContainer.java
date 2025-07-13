// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.elbow_subsystem.ElbowElevationRotationCommand;
import frc.robot.subsystems.elbow_subsystem.ElbowSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToPosCommand;

// Auto scoring imports
import frc.robot.commands.AutoPickup;
import frc.robot.commands.AutoPlace;
import frc.robot.commands.AutoPlace.Node;
import scoringcontroller.CommandCustomController;

public class RobotContainer {
	// Speed limits
	private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
	private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
																																										// second, max angular velocity
	private double MaxControlSpeed = 1;

	// Auto scoring variables
	private int level = 0;
	private AutoPlace.HexSide hexSide = AutoPlace.HexSide.A;
	private AutoPlace.Side side = AutoPlace.Side.one;
	private boolean level1Pickup = false;

	// Setting up bindings for necessary control of the swerve drive platform
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
																																// motors
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	// Telemetry
	private final Telemetry logger = new Telemetry(MaxSpeed);

	// Controllers
	private final CommandXboxController driverController = new CommandXboxController(0);
	private final CommandCustomController CustomController = new CommandCustomController(1);
	private final CommandXboxController operatorController = new CommandXboxController(2);
	private static final double XBOX_DEADBAND = 0.1;

	// Create Subsystems
	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
	public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
	public final ElbowSubsystem elbowSubsystem = new ElbowSubsystem();

	// Path follower
	private final SendableChooser<Command> autoChooser;

	/**
	 * RobotContainer constructor initializes the robot.
	 */
	public RobotContainer() {
		// Register the named commands for auto
		registerCommands();
		autoChooser = AutoBuilder.buildAutoChooser("ScoreL1FromCenter"); // @TODO add auto program
		SmartDashboard.putData("Auto Mode", autoChooser);

		configureBindings();
	}

	/**
	 * Configure all bindings for the robot's controls.
	 */
	private void configureBindings() {
		//// ----------------- Driving Commands -----------------
		// Note that X is defined as forward according to WPILib convention,
		// and Y is defined as to the left according to WPILib convention.
		drivetrain.setDefaultCommand(
				// Drivetrain will execute this command periodically
				drivetrain.applyRequest(() -> drive
						.withVelocityX(MathUtil.applyDeadband(-driverController.getLeftY(), XBOX_DEADBAND) * MaxSpeed
								* (driverController.getRightTriggerAxis() * 2 + 1)) // Drive forward with negative Y (forward)
						.withVelocityY(MathUtil.applyDeadband(-driverController.getLeftX(), XBOX_DEADBAND) * MaxSpeed
								* (driverController.getRightTriggerAxis() * 2 + 1)) // Drive left with negative X (left)
						.withRotationalRate(MathUtil.applyDeadband(-driverController.getRightX(), XBOX_DEADBAND) * MaxAngularRate
								* (driverController.getRightTriggerAxis() * 2 + 1)) // Drive counterclockwise with negative X (left)
				));

		// Reset the field-centric heading on left bumper press
		driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

		// Idle while the robot is disabled. This ensures the configured
		// neutral mode is applied to the drive motors while disabled.
		final var idle = new SwerveRequest.Idle();
		RobotModeTriggers.disabled().whileTrue(
				drivetrain.applyRequest(() -> idle).ignoringDisable(true));

		driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
		driverController.b().whileTrue(drivetrain.applyRequest(
				() -> point.withModuleDirection(
						new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

		// Run SysId routines when holding back/start and X/Y.
		// Note that each routine should be run exactly once in a single log.
		driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
		driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
		driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

		drivetrain.registerTelemetry(logger::telemeterize);

		//// ----------------- Elevator Commands ----------------
		driverController.a().onTrue(new ElevatorToPosCommand(ElevatorSubsystem.LEVEL1_POSITION, elevatorSubsystem));
		driverController.b().onTrue(new ElevatorToPosCommand(ElevatorSubsystem.LEVEL2_POSITION, elevatorSubsystem));
		driverController.x().onTrue(new ElevatorToPosCommand(ElevatorSubsystem.LEVEL3_POSITION, elevatorSubsystem));
		driverController.y().onTrue(new ElevatorToPosCommand(ElevatorSubsystem.LEVEL4_POSITION, elevatorSubsystem));

		//// ----------------- Elbow Commands ----------------
		driverController.leftBumper().onTrue(new ElbowElevationRotationCommand(0.5, 0.5, elbowSubsystem));
		driverController.rightBumper().onTrue(new ElbowElevationRotationCommand(0, 0, elbowSubsystem));

		////// ---------------- Automated Commands ----------------
		//// Choosing where to score on Custom Controller
		// CustomController.bt1().onTrue(new RunCommand(() -> {
		// 	hexSide = AutoPlace.HexSide.A;
		// 	side = AutoPlace.Side.one;
		// }));
		// CustomController.bt2().onTrue(new RunCommand(() -> {
		// 	hexSide = AutoPlace.HexSide.A;
		// 	side = AutoPlace.Side.two;
		// }));
		// CustomController.bt3().onTrue(new RunCommand(() -> {
		// 	hexSide = AutoPlace.HexSide.B;
		// 	side = AutoPlace.Side.one;
		// }));
		// CustomController.bt4().onTrue(new RunCommand(() -> {
		// 	hexSide = AutoPlace.HexSide.B;
		// 	side = AutoPlace.Side.two;
		// }));
		// CustomController.bt5().onTrue(new RunCommand(() -> {
		// 	hexSide = AutoPlace.HexSide.C;
		// 	side = AutoPlace.Side.one;
		// }));
		// CustomController.bt6().onTrue(new RunCommand(() -> {
		// 	hexSide = AutoPlace.HexSide.C;
		// 	side = AutoPlace.Side.two;
		// }));
		// CustomController.bt7().onTrue(new RunCommand(() -> {
		// 	hexSide = AutoPlace.HexSide.D;
		// 	side = AutoPlace.Side.one;
		// }));
		// CustomController.bt8().onTrue(new RunCommand(() -> {
		// 	hexSide = AutoPlace.HexSide.D;
		// 	side = AutoPlace.Side.two;
		// }));
		// CustomController.bt9().onTrue(new RunCommand(() -> {
		// 	hexSide = AutoPlace.HexSide.E;
		// 	side = AutoPlace.Side.one;
		// }));
		// CustomController.bt10().onTrue(new RunCommand(() -> {
		// 	hexSide = AutoPlace.HexSide.E;
		// 	side = AutoPlace.Side.two;
		// }));
		// CustomController.bt11().onTrue(new RunCommand(() -> {
		// 	hexSide = AutoPlace.HexSide.F;
		// 	level = 1;
		// }));
		// CustomController.bt12().onTrue(new RunCommand(() -> {
		// 	hexSide = AutoPlace.HexSide.F;
		// 	level = 2;
		// }));
		// CustomController.bt16().onTrue(new RunCommand(() -> {
		// 	level = 1;
		// }));
		// CustomController.bt17().onTrue(new RunCommand(() -> {
		// 	level = 2;
		// }));
		// CustomController.bt18().onTrue(new RunCommand(() -> {
		// 	level = 3;
		// }));
		// CustomController.bt19().onTrue(new RunCommand(() -> {
		// 	level = 4;
		// }));

		hexSide = AutoPlace.HexSide.C;
		side = AutoPlace.Side.one;
		level = 1;

		driverController.back().whileTrue(new InstantCommand(
			()-> System.out.println("Back btn pressed")
		));


		// Autoplace command (Allow operator to also place)
		driverController.back().whileTrue(new AutoPlace(drivetrain,
		elevatorSubsystem, elbowSubsystem,
		new Node(level, hexSide, side)));
		// operatorController.rightBumper().whileTrue(new AutoPlace(drivetrain,
		// elevatorSubsystem, elbowSubsystem,
		// new Node(level, hexSide, side)));

		// // Auto pickup command
		// // If wanting to pickup to score for level 1, press A, otherwise press Y
		// operatorController.y().whileTrue(new RunCommand(() -> level1Pickup = false));
		// operatorController.a().whileTrue(new RunCommand(() -> level1Pickup = true));
		// operatorController.leftBumper().whileTrue(new AutoPickup(drivetrain,
		// elevatorSubsystem,
		// () -> AutoPickup.getCoralSide(drivetrain.getState().Pose), level1Pickup));

	}

	public Command getAutonomousCommand() {
		/* Run the path selected from the auto chooser */
		return autoChooser.getSelected();
	}

	private void registerCommands() {
		// // Register the commands here
		// NamedCommands.registerCommand("ElevatorHomingCommand", new
		// ElevatorHomingCommand(elevatorSubsystem));
		// NamedCommands.registerCommand("ElevatorToLVL1",
		// new ElevatorToPosCommand(ElevatorSubsystem.level1Position,
		// elevatorSubsystem));
		// NamedCommands.registerCommand("ElevatorToLVL2",
		// new ElevatorToPosCommand(ElevatorSubsystem.level2Position,
		// elevatorSubsystem));
		// NamedCommands.registerCommand("ElevatorToLVL3",
		// new ElevatorToPosCommand(ElevatorSubsystem.level3Position,
		// elevatorSubsystem));
		// NamedCommands.registerCommand("ElevatorToLVL4",
		// new ElevatorToPosCommand(ElevatorSubsystem.level4Position,
		// elevatorSubsystem));
	}
}
