// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
//import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.auto.NamedCommands;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.ConditionalCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.custom_lib.CommandLogitecController;
//import frc.robot.TunerConstants;
import frc.robot.subsystems.ArmMotorSubsystem;
import frc.robot.subsystems.ArmThenIntakeCommand;
import frc.robot.subsystems.IntakeMotorSubsystem;
//import frc.robot.subsystems.MotorTesting.IntakeCommandFactory;
import scoringcontroller.CommandCustomController;


public class RobotContainer {
    
	
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

	public ArmMotorSubsystem getArmMotorSubsystem() {
		return armMotorSS;
	}


	/**
	 * RobotContainer constructor initializes the robot.
	 */
	public RobotContainer() {
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

		operatorController.a().onTrue(Commands.runOnce(() -> armMotorSS.increaseVoltage()));
		operatorController.b().onTrue(Commands.runOnce(() -> armMotorSS.decreaseVoltage()));

	}

}