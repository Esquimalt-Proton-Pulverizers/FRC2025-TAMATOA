// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArmToPosCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.elbow_subsystem.ElbowElevationRotationCommand;
import frc.robot.subsystems.elbow_subsystem.ElbowSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hang.HangingSubsystem;
import frc.robot.subsystems.intake_subsystem.IntakeSubsystem;
import frc.robot.commands.AutoPickup;
import frc.robot.commands.AutoPlace;
import frc.robot.commands.AutoPlace.Node;
import scoringcontroller.CommandCustomController;


public class RobotContainer {
    // Swerve Drive variables
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double MaxControlSpeed = 1.0;
    private double MinControlSpeed = 0.5;
    private double throttle;

	// Setting up bindings for necessary control of the swerve drive platform
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	// private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	// Telemetry
	private final Telemetry logger = new Telemetry(MaxSpeed);

	// Controllers
	private final CommandXboxController driverController = new CommandXboxController(0);
	private final CommandXboxController operatorController = new CommandXboxController(1);
    private final CommandCustomController CustomController = new CommandCustomController(2);
	private static final double XBOX_DEADBAND = 0.1;

	// Create Subsystems
	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
	public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
	public final ElbowSubsystem elbowSubsystem = new ElbowSubsystem();
	public final HangingSubsystem hanger = new HangingSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();


    // Auto scoring variables
	private int level = 0;
	private AutoPlace.HexSide hexSide = AutoPlace.HexSide.A;
	private AutoPlace.Side side = AutoPlace.Side.one;

	// Path follower
	private final SendableChooser<Command> autoChooser;


	/**
	 * RobotContainer constructor initializes the robot.
	 */
	public RobotContainer() {
		// Register the named commands for auto
		registerCommands();
        configureBindings();
		autoChooser = AutoBuilder.buildAutoChooser("ScoreL1FromCenter"); // @TODO add auto program
		SmartDashboard.putData("Auto Mode", autoChooser);
    }


	/**
	 * Configure all bindings for the robot's controls.
	 */
	private void configureBindings() {
        /////////////////////////////////////////////////////////
        ////// ------------- Driver Controls ------------- //////
        /////////////////////////////////////////////////////////
        
		//// ----------------- Driving Commands -----------------
        // Set drive speed (Throttle Control)
        throttle = (driverController.getRightTriggerAxis() / 2.0) + MinControlSpeed;
        // Drive Controls
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * throttle) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * throttle) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * throttle) // Drive counterclockwise with negative X (left)
            )
        );

		// Reset the field-centric heading on left bumper press
		driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Brake Mode - Stop robot from being moved
		driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));


        //// ----------------- Hanging Controls -----------------
		driverController.povUp().onTrue(hanger.extend());
        driverController.povDown().onTrue(hanger.retract());
		driverController.leftBumper().onTrue(hanger.intake());
        driverController.leftBumper().onFalse(hanger.stop());
        driverController.back().onTrue(hanger.manualRetract());
        driverController.back().onFalse(hanger.resetWinch());


        /////////////////////////////////////////////////////////
        ////// ------------ Operator Controls ------------ //////
        /////////////////////////////////////////////////////////

        //// ------------------- Arm Controls -------------------
        double curElbowElevationPos = elbowSubsystem.getElevationPos();
        double curElbowRotationPos = elbowSubsystem.getRotationPos();
        double curElevatorPos = elevatorSubsystem.getPosition();

        operatorController.leftBumper().onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LOW_POSITION, ElbowSubsystem.INTAKE_POS, curElbowElevationPos, curElbowRotationPos, curElevatorPos));
        operatorController.a().onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LEVEL1_POSITION, ElbowSubsystem.LOW_POS, curElbowElevationPos, curElbowRotationPos, curElevatorPos));
        operatorController.b().onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LEVEL2_POSITION, ElbowSubsystem.MIDS_POS, curElbowElevationPos, curElbowRotationPos, curElevatorPos));
        operatorController.x().onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LEVEL3_POSITION, ElbowSubsystem.MIDS_POS, curElbowElevationPos, curElbowRotationPos, curElevatorPos));
        operatorController.y().onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LEVEL4_POSITION, ElbowSubsystem.HIGH_POS, curElbowElevationPos, curElbowRotationPos, curElevatorPos));
		operatorController.start().onTrue(elbowSubsystem.resetEncoderCommand());

        //// ---------------- Intake Commands ----------------
        // @TODO: Add Intake Commands
        // Left Trigger Intake, Right Trigger Outake

		//// ----------------- Elbow Commands ----------------
		operatorController.povUp().onTrue(new ElbowElevationRotationCommand(0.5, 0.5, elbowSubsystem));
		operatorController.povDown().onTrue(new ElbowElevationRotationCommand(0, 0, elbowSubsystem));


        /////////////////////////////////////////////////////////
        ////// ----------- Automated Controls ----------- //////
        /////////////////////////////////////////////////////////
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

		// hexSide = AutoPlace.HexSide.C;
		// side = AutoPlace.Side.one;
		// level = 1;

		// driverController.back().whileTrue(new InstantCommand(
		// 	()-> System.out.println("Back btn pressed")
		// ));


		// // Auto-place command
		// driverController.a().whileTrue(new AutoPlace(drivetrain,
        //     elevatorSubsystem, elbowSubsystem,
        //     new Node(level, hexSide, side)));
        // // Auto-score command
        // driverController.leftBumper().whileTrue(new AutoPickup(drivetrain,
        //     elevatorSubsystem,
        //     () -> AutoPickup.getCoralSide(drivetrain.getState().Pose), false));
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