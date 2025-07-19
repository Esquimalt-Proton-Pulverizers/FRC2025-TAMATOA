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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    private double MinControlSpeed = 1.0;
    private double throttle;

	// Setting up bindings for necessary control of the swerve drive platform
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

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

    // Manual Movement
    public double elevatorPosChange;
    public final double ELBOW_MOVEMENT_PER_CLICK = 0.5;

    // Manual Override and Encoder Reset
    private boolean manualOverride = false;
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
        throttle = (driverController.getRightTriggerAxis() * 2.0) + MinControlSpeed;
        throttle = throttle > MaxControlSpeed ? MaxControlSpeed : throttle;
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

        // Positions of Elevator and Elbow
        double curElbowElevationPos = elbowSubsystem.getElevationPos();
        double curElbowRotationPos = elbowSubsystem.getRotationPos();
        double curElevatorPos = elevatorSubsystem.getPosition();

        //// ------------------- Arm Controls -------------------
        operatorController.leftBumper().onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LOW_POSITION, ElbowSubsystem.INTAKE_POS, curElbowElevationPos, curElbowRotationPos, curElevatorPos));
        operatorController.a().onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LEVEL1_POSITION, ElbowSubsystem.LOW_POS, curElbowElevationPos, curElbowRotationPos, curElevatorPos));
        operatorController.b().onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LEVEL2_POSITION, ElbowSubsystem.MIDS_POS, curElbowElevationPos, curElbowRotationPos, curElevatorPos));
        operatorController.x().onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LEVEL3_POSITION, ElbowSubsystem.MIDS_POS, curElbowElevationPos, curElbowRotationPos, curElevatorPos));
        operatorController.y().onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LEVEL4_POSITION, ElbowSubsystem.HIGH_POS, curElbowElevationPos, curElbowRotationPos, curElevatorPos));
		operatorController.start().onTrue(elbowSubsystem.resetEncoderCommand());
        operatorController.rightBumper().onTrue(new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, ElevatorSubsystem.LOW_POSITION, ElbowSubsystem.HOMING_POS, curElbowElevationPos, curElbowRotationPos, curElevatorPos));


        //// ---------------- Intake Commands ----------------
        operatorController.povUp().onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.intake()));
        operatorController.povDown().onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.outtake()));
        operatorController.povUp().onFalse(intakeSubsystem.runOnce(() -> intakeSubsystem.stop()));
        operatorController.povDown().onFalse(intakeSubsystem.runOnce(() -> intakeSubsystem.stop()));

        operatorController.leftTrigger().whileTrue(new InstantCommand(
			()-> System.out.println("Left Trigger pressed")
		));

        //// --------------- Elevator Commands ---------------
        elevatorPosChange = operatorController.getRightY();
        if (Math.abs(elevatorPosChange) > 0.1) {
            elevatorPosChange = elevatorSubsystem.getPosition() + elevatorPosChange;
            elevatorSubsystem.runOnce(() -> new ElevatorToPosCommand(elevatorPosChange, elevatorSubsystem, manualOverride));
        }

		// //// ----------------- Elbow Commands ----------------
		// operatorController.povUp().onTrue(new ElbowElevationRotationCommand(curElbowElevationPos + ELBOW_MOVEMENT_PER_CLICK, curElbowRotationPos, elbowSubsystem, manualOverride));
		// operatorController.povDown().onTrue(new ElbowElevationRotationCommand(curElbowElevationPos - ELBOW_MOVEMENT_PER_CLICK, curElbowRotationPos, elbowSubsystem, manualOverride));
		// operatorController.povLeft().onTrue(new ElbowElevationRotationCommand(curElbowElevationPos, curElbowRotationPos - ELBOW_MOVEMENT_PER_CLICK, elbowSubsystem, manualOverride));
		// operatorController.povRight().onTrue(new ElbowElevationRotationCommand(curElbowElevationPos, curElbowRotationPos + ELBOW_MOVEMENT_PER_CLICK, elbowSubsystem, manualOverride));

        // // //// ---------------- Manual Override ----------------
        // // operatorController.back().onTrue(Commands.runOnce(() -> manualOverride = !manualOverride)); // Enable/ Disable hard limits
        // operatorController.start().onTrue(Commands.runOnce(() -> encoderReset = true)); // Reset Encoder Positions for Elevator and Elbow
        // if (encoderReset) {
        //     ElevatorSubsystem.resetEncoder();
        //     ElbowSubsystem.resetEncoder();
        //     manualOverride = false;
        // }


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