// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArmToPosCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.elbow_subsystem.ElbowElevationRotationCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToPosCommand;
import frc.robot.subsystems.hang.HangingSubsystem;
import frc.robot.subsystems.intakeSubsystem.IntakeSubsystem;
import frc.robot.commands.AutoPickup;
import frc.robot.commands.AutoPlace;
import frc.robot.commands.AutoPlace.Node;
import scoringcontroller.CommandCustomController;


public class RobotContainer {
    // Swerve Drive variables
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double MaxControlSpeed = 3.0;
	private final double DRIVE_DEADBAND = 0.0;
	private final double TURBO_BUTTON_MULTIPLE = 2.0;

	// Setting up bindings for necessary control of the swerve drive platform
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(MaxSpeed * DRIVE_DEADBAND).withRotationalDeadband(MaxAngularRate * DRIVE_DEADBAND) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

	// Telemetry
	private final Telemetry logger = new Telemetry(MaxSpeed);

	// Controllers
	private final CommandXboxController driverController = new CommandXboxController(0);
	private final CommandGenericHID operatorController = new CommandGenericHID(1);
    private final CommandCustomController CustomController = new CommandCustomController(2);
	private static final double XBOX_DEADBAND = 0.05;
	public final double RIGHT_TRIGGER_OFFSET = 1; //changes the right trigger range to be 1-2 instead of 0-1

	// Create Subsystems
	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
	public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
	public final HangingSubsystem hanger = new HangingSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    // Manual Movement
    public final double ELEVATOR_MOVEMENT_PER_CLICK = 1.0;
    public final double ELBOW_ELEVATION_MOVEMENT_PER_CLICK = 1.0;
	public final double ELBOW_ROTATION_MOVEMENT_PER_CLICK = 10.0;

    // Manual Override and Encoder Reset
    public static boolean manualOverride = false;
    private boolean encoderReset = false;

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
        configureBindings();
		autoChooser = AutoBuilder.buildAutoChooser("Center - Score L1A"); // Default auto program to run
		SmartDashboard.putData("Auto Mode", autoChooser);
    }
	private static double applyDeadband(double value) {
        if (Math.abs(value) < XBOX_DEADBAND) {
            return 0.0;
        }

        // Rescale so the output goes from 0 to 1 outside the deadband
        double sign = Math.signum(value);
        double adjusted = (Math.abs(value) - XBOX_DEADBAND) / (1.0 - XBOX_DEADBAND);
        return sign * adjusted;
    }

	/**
	 * Configure all bindings for the robot's controls.
	 */
	private void configureBindings() {
        /////////////////////////////////////////////////////////
        ////// ------------- Driver Controls ------------- //////
        /////////////////////////////////////////////////////////
        
		//// ----------------- Driving Commands -----------------
        // Drive Controls
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(applyDeadband(-driverController.getLeftY()) * ((driverController.getRightTriggerAxis() + RIGHT_TRIGGER_OFFSET) * TURBO_BUTTON_MULTIPLE) ) // Drive forward with negative Y (forward)
        //             .withVelocityY(applyDeadband(-driverController.getLeftX()) * ((driverController.getRightTriggerAxis() + RIGHT_TRIGGER_OFFSET)  * TURBO_BUTTON_MULTIPLE )) // Drive left with negative X (left)
        //             .withRotationalRate(applyDeadband(-driverController.getRightX()) * ((driverController.getRightTriggerAxis() + RIGHT_TRIGGER_OFFSET)  * TURBO_BUTTON_MULTIPLE) ) // Drive counterclockwise with negative X (left)
        //     )
        // );

		// // Reset the field-centric heading on left bumper press
		// driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // // Brake Mode - Stop robot from being moved
		// driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));


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
        
        //// -------------------- Cancel All --------------------
        operatorController.button(12).onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));

        // Positions of Elevator and Elbow
        double curElevatorPos = elevatorSubsystem.getPosition();

        //// ---------------- Intake Commands ----------------
        operatorController.button(7).onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.intake()));  // Left Trigger	
        operatorController.button(8).onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.outtake())); // Right Trigger	
        operatorController.button(7).onFalse(intakeSubsystem.runOnce(() -> intakeSubsystem.stop()));   // Left Trigger	
        operatorController.button(8).onFalse(intakeSubsystem.runOnce(() -> intakeSubsystem.stop()));   // Right Trigger	

        //// --------------- Elevator Commands ---------------
        operatorController.button(11).onTrue(Commands.runOnce(()->elevatorSubsystem.manualMove(ELEVATOR_MOVEMENT_PER_CLICK), elevatorSubsystem));  // Left Stick Button
        operatorController.button(12).onTrue(Commands.runOnce(()->elevatorSubsystem.manualMove(-ELEVATOR_MOVEMENT_PER_CLICK), elevatorSubsystem)); // Right Stick Button

	}

	public Command getAutonomousCommand() {
		/* Run the path selected from the auto chooser */
		return autoChooser.getSelected();
	}
}