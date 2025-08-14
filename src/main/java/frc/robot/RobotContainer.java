// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;
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
import frc.custom_lib.CommandLogitecController;
import frc.robot.TunerConstants;
import frc.robot.subsystems.ArmMotorSubsystem;
import frc.robot.subsystems.ArmThenIntakeCommand;
import frc.robot.subsystems.IntakeMotorSubsystem;
//import frc.robot.subsystems.MotorTesting.IntakeCommandFactory;
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
	private final CommandLogitecController operatorController = new CommandLogitecController(1);
    private final CommandCustomController CustomController = new CommandCustomController(2);
	private static final double XBOX_DEADBAND = 0.05;
	public final double RIGHT_TRIGGER_OFFSET = 1; //changes the right trigger range to be 1-2 instead of 0-1

	// Create Subsystems
    // public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	public final IntakeMotorSubsystem intakeMotorSS = new IntakeMotorSubsystem();
	public final ArmMotorSubsystem armMotorSS = new ArmMotorSubsystem();


    // Manual Movement
    public final double ELEVATOR_MOVEMENT_PER_CLICK = 1.0;
    public final double ELBOW_ELEVATION_MOVEMENT_PER_CLICK = 1.0;
	public final double ELBOW_ROTATION_MOVEMENT_PER_CLICK = 10.0;

    // Manual Override and Encoder Reset
    public static boolean manualOverride = false;
    private boolean encoderReset = false;


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
  
		//Motor Testing
		operatorController.povUp().onTrue(Commands.runOnce(() -> intakeMotorSS.kSlotIncrementOne()));
		operatorController.povDown().onTrue(Commands.runOnce(() -> intakeMotorSS.kSlotDecrementOne()));

		operatorController.povLeft().onTrue(Commands.runOnce(() -> intakeMotorSS.driveMotorToPos(10, ArmMotorSubsystem.kSlot)));
		operatorController.povRight().onTrue(Commands.runOnce(() -> intakeMotorSS.driveMotorToPos(0, ArmMotorSubsystem.kSlot)));

		operatorController.leftTrigger().onTrue(new ArmThenIntakeCommand(armMotorSS,intakeMotorSS,10,10)); 

		//operatorController.leftTrigger().onTrue(Commands.runOnce(() -> armMotorSS.driveMotorToPos(10,ArmMotorSubsystem.kSlot))); 
		operatorController.rightTrigger().onTrue(Commands.runOnce(() -> armMotorSS.driveMotorToPos(0,ArmMotorSubsystem.kSlot)));

		// operatorController.rightBumper().onTrue(Commands.runOnce(() -> motorTesting.posDecrease()));
		// operatorController.leftBumper().onTrue(Commands.runOnce(() -> motorTesting.posIncrease()));
		// // operatorController.rightStick().onTrue(motorTesting.intakeToPos());
		// operatorController.a().onTrue(IntakeCommandFactory.createIntakeToPosCommand(motorTesting));

		// operatorController.button(1).onTrue(Commands.runOnce(() -> motorTesting.driveMotorToPos(10,0)));  	
		// operatorController.button(2).onTrue(Commands.runOnce(() -> motorTesting.driveMotorToPos(10,1)));  	
		// operatorController.button(3).onTrue(Commands.runOnce(() -> motorTesting.driveMotorToPos(10,2)));  
		// operatorController.button(4).onTrue(Commands.runOnce(() -> motorTesting.driveMotorToPos(10,3)));  	
		// operatorController.button(5).onTrue(Commands.runOnce(() -> motorTesting.driveMotorToPos(0,0)));  	
		// operatorController.button(6).onTrue(Commands.runOnce(() -> motorTesting.driveMotorToPos(0,1)));	
		// operatorController.button(7).onTrue(Commands.runOnce(() -> motorTesting.driveMotorToPos(0,2)));  	
		// operatorController.button(8).onTrue(Commands.runOnce(() -> motorTesting.driveMotorToPos(0,3)));  	




        //// --------------- Elevator Commands ---------------
        // operatorController.button(11).onTrue(Commands.runOnce(()->elevatorSubsystem.manualMove(ELEVATOR_MOVEMENT_PER_CLICK), elevatorSubsystem));  // Left Stick Button
        // operatorController.button(12).onTrue(Commands.runOnce(()->elevatorSubsystem.manualMove(-ELEVATOR_MOVEMENT_PER_CLICK), elevatorSubsystem)); // Right Stick Button
		// operatorController.button(9).onTrue(new ElevatorToPosCommand(20.0, elevatorSubsystem));    
		// operatorController.button(10).onTrue(new ElevatorInchUpCommand(1.0, elevatorSubsystem));    
		// operatorController.button(9).onTrue(new ElevatorInchUpCommand(-10.0, elevatorSubsystem));    

	}

	public Command getAutonomousCommand() {
		/* Run the path selected from the auto chooser */
		return autoChooser.getSelected();
	}
}