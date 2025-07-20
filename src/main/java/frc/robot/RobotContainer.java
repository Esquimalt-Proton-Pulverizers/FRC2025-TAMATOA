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
import frc.robot.subsystems.elbow_subsystem.ElbowSubsystem;
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
	public final ElbowSubsystem elbowSubsystem = new ElbowSubsystem();
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
		registerCommands();
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
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(applyDeadband(-driverController.getLeftY()) * ((driverController.getRightTriggerAxis() + RIGHT_TRIGGER_OFFSET) * TURBO_BUTTON_MULTIPLE) ) // Drive forward with negative Y (forward)
                    .withVelocityY(applyDeadband(-driverController.getLeftX()) * ((driverController.getRightTriggerAxis() + RIGHT_TRIGGER_OFFSET)  * TURBO_BUTTON_MULTIPLE )) // Drive left with negative X (left)
                    .withRotationalRate(applyDeadband(-driverController.getRightX()) * ((driverController.getRightTriggerAxis() + RIGHT_TRIGGER_OFFSET)  * TURBO_BUTTON_MULTIPLE) ) // Drive counterclockwise with negative X (left)
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
        
        //// -------------------- Cancel All --------------------
        operatorController.button(12).onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));

        // Positions of Elevator and Elbow
        double curElbowElevationPos = elbowSubsystem.getElevationPos();
        double curElbowRotationPos = elbowSubsystem.getRotationPos();
        double curElevatorPos = elevatorSubsystem.getPosition();

        //// ------------------- Arm Controls -------------------
        operatorController.button(5).onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LOW_POSITION, ElbowSubsystem.INTAKE_POS, curElbowElevationPos, curElbowRotationPos, curElevatorPos));  // Left Bumper
        operatorController.button(2).onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LEVEL1_POSITION, ElbowSubsystem.LOW_POS, curElbowElevationPos, curElbowRotationPos, curElevatorPos));  // A
        operatorController.button(1).onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LEVEL2_POSITION, ElbowSubsystem.MIDS_POS, curElbowElevationPos, curElbowRotationPos, curElevatorPos)); // X
        operatorController.button(3).onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LEVEL3_POSITION, ElbowSubsystem.MIDS_POS, curElbowElevationPos, curElbowRotationPos, curElevatorPos)); // B
        operatorController.button(4).onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LEVEL4_POSITION, ElbowSubsystem.HIGH_POS, curElbowElevationPos, curElbowRotationPos, curElevatorPos)); // Y
        operatorController.button(6).onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LOW_POSITION, ElbowSubsystem.HOMING_POS, curElbowElevationPos, curElbowRotationPos, curElevatorPos));  // Right Bumper
        operatorController.button(9).onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LOCK_POSITION, ElbowSubsystem.HOMING_POS, curElbowElevationPos, curElbowRotationPos, curElevatorPos)); // Back Button
        // operatorController.povRight().onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.CORAL_STATION_POSITION, ElbowSubsystem.CORAL_POS, curElbowElevationPos, curElbowRotationPos, curElevatorPos));


        //// ---------------- Intake Commands ----------------
        operatorController.button(7).onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.intake()));  // Left Trigger	
        operatorController.button(8).onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.outtake())); // Right Trigger	
        operatorController.button(7).onFalse(intakeSubsystem.runOnce(() -> intakeSubsystem.stop()));   // Left Trigger	
        operatorController.button(8).onFalse(intakeSubsystem.runOnce(() -> intakeSubsystem.stop()));   // Right Trigger	

        //// --------------- Elevator Commands ---------------
        operatorController.button(11).onTrue(Commands.runOnce(()->elevatorSubsystem.manualMove(ELEVATOR_MOVEMENT_PER_CLICK), elevatorSubsystem));  // Left Stick Button
        operatorController.button(12).onTrue(Commands.runOnce(()->elevatorSubsystem.manualMove(-ELEVATOR_MOVEMENT_PER_CLICK), elevatorSubsystem)); // Right Stick Button

		//// ----------------- Elbow Commands ----------------
		operatorController.povUp().onTrue(Commands.runOnce(()->elbowSubsystem.manualMove(ELBOW_ELEVATION_MOVEMENT_PER_CLICK, 0.0), elbowSubsystem));
		operatorController.povDown().onTrue(Commands.runOnce(()->elbowSubsystem.manualMove(-ELBOW_ELEVATION_MOVEMENT_PER_CLICK, 0.0), elbowSubsystem));
		operatorController.povLeft().onTrue(Commands.runOnce(()->elbowSubsystem.manualMove(0.0, -ELBOW_ROTATION_MOVEMENT_PER_CLICK), elbowSubsystem));
		operatorController.povRight().onTrue(Commands.runOnce(()->elbowSubsystem.manualMove(0.0, ELBOW_ROTATION_MOVEMENT_PER_CLICK), elbowSubsystem));

		//// -------- Manual Override + Encoder Reset --------
		// If Manual Override is false, become true
		// If Manual Override is true, reset encoder positions, and then become false
        operatorController.button(10).onTrue(Commands.runOnce(() -> 
			new ConditionalCommand(
				new ParallelCommandGroup(
					Commands.runOnce(() -> ElevatorSubsystem.resetEncoder()),
					Commands.runOnce(() -> ElbowSubsystem.resetEncoder()),
					Commands.runOnce(() -> manualOverride = false)
				), 
				Commands.runOnce(() -> manualOverride = true),
				() -> manualOverride)
			));


        /////////////////////////////////////////////////////////
        ////// ----------- Automated Controls ----------- ///////
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
		// Register the commands here
		NamedCommands.registerCommand("ArmToLevel1", new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, 
                ElevatorSubsystem.LEVEL1_POSITION, ElbowSubsystem.LOW_POS, ElbowSubsystem.START_POS_ELEVATION, 
                ElbowSubsystem.START_POS_ROTATION, 0.0));
        NamedCommands.registerCommand("ArmHomingAfterLevel1", new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, 
                elevatorSubsystem.getPosition(), ElbowSubsystem.HOMING_POS, elbowSubsystem.getElevationPos(), 
                elbowSubsystem.getRotationPos(), ElevatorSubsystem.LEVEL1_POSITION));
        NamedCommands.registerCommand("ArmToLevel2", new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, 
                ElevatorSubsystem.LEVEL2_POSITION, ElbowSubsystem.MIDS_POS, ElbowSubsystem.START_POS_ELEVATION, 
                ElbowSubsystem.START_POS_ROTATION, ElevatorSubsystem.LOW_POSITION));
        NamedCommands.registerCommand("ArmToLevel3", new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, 
                ElevatorSubsystem.LEVEL3_POSITION, ElbowSubsystem.MIDS_POS, ElbowSubsystem.START_POS_ELEVATION, 
                ElbowSubsystem.START_POS_ROTATION, ElevatorSubsystem.LOW_POSITION));
        NamedCommands.registerCommand("ArmToLevel4", new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, 
                ElevatorSubsystem.LEVEL4_POSITION, ElbowSubsystem.HIGH_POS, ElbowSubsystem.START_POS_ELEVATION, 
                ElbowSubsystem.START_POS_ROTATION, ElevatorSubsystem.LOW_POSITION));
        NamedCommands.registerCommand("CoralIntake", intakeSubsystem.runOnce(() -> intakeSubsystem.intake()));
        NamedCommands.registerCommand("CoralOutake", intakeSubsystem.runOnce(() -> intakeSubsystem.outtake()));
        NamedCommands.registerCommand("IntakeStop", intakeSubsystem.runOnce(() -> intakeSubsystem.stop()));
	}
}