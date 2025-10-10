// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import java.util.Set;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.hang.HangingSubsystem;
import frc.robot.subsystems.intakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.scoring_subsystem.ManualScoringControlCommand;
import frc.robot.subsystems.scoring_subsystem.ScoringSubsystem;
import frc.robot.subsystems.scoring_subsystem.ScoringSubsystem.Position;
import frc.robot.subsystems.scoring_subsystem.differential.DifferentialSubsystem;
import frc.robot.subsystems.scoring_subsystem.differential.WristFlipCommand;
import frc.robot.subsystems.scoring_subsystem.elevator.ElevatorSubsystem;
import frc.robot.commands.AutoPlace;
import frc.robot.commands.AutoScoringPathBuilder;
import frc.robot.commands.AutoPlace.Node;
import scoringcontroller.CommandCustomController;
import frc.robot.subsystems.lights.LEDlights;

public class RobotContainer {
    // Swerve Drive Controls
	private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts top speed possible at 12 volts
	private final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second, max angular velocity
	private final double MAX_CONTROL_SPEED = 1; //max speed the driver can go in x or y in m/s
	private final double TURBO_MULTIPLE = 1.3; // technically a divider for how slow it is pre turbo since it is limited to the max control speed ... 1.0 disables it

	// Auto scoring variables
	private Position autoScoringPosition = Position.SCORE_L1;
	private AutoPlace.HexSide hexSide = AutoPlace.HexSide.A;
	private AutoPlace.Side side = AutoPlace.Side.one;

	// Setting up bindings for necessary control of the swerve drive platform
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(0).withRotationalDeadband(0) // don't apply deadband here, it ends up being jerky apply it in the request supplier
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

	// Telemetry
	private final Telemetry logger = new Telemetry(MaxSpeed);

	// Controllers
	private final CommandXboxController driverController = new CommandXboxController(0);
	private final CommandGenericHID operatorController = new CommandGenericHID(1);
    private final CommandCustomController CustomController = new CommandCustomController(2);
	private static final double XBOX_DEADBAND = 0.09;
	// public final double TRIGGER_OFFSET = 1; //changes the right trigger range to be 1-2 instead of 0-1

	// Create Subsystems
	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); //this should create drivetrain and configure the Autobuilder settings
	public final HangingSubsystem hanger = new HangingSubsystem(false);
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(false);
	public final ScoringSubsystem scoringSubsystem = new ScoringSubsystem(true);

    // Manual Movement
    public final double ELEVATOR_MOVEMENT_PER_CLICK = 1.0;
    public final double ELBOW_ELEVATION_MOVEMENT_PER_CLICK = 1.0;
	public final double ELBOW_ROTATION_MOVEMENT_PER_CLICK = 10.0;

    // Manual Override and Encoder Reset
    public static boolean manualOverride = false;
    private boolean encoderReset = false;
	private RobotModes robotMode = RobotModes.CoralMode;

	public enum RobotModes {
		CoralMode,
		AlgaeMode,
		ManualMoveMode,
		HangingMode;
	}


	// Path follower
	private final SendableChooser<Command> autoChooser;
	// LED Lights
	public final LEDlights ledLights = new LEDlights();



	/**
	 * RobotContainer constructor initializes the robot.
	 */
	public RobotContainer() {
		// Register the named commands for auto
		registerCommands();
		configureDriveBindings(true);//false just disables driving without breaking limelight
		//configureOperatorBindingsBrandon();
		configureOperatorBindingsColin();
		configureAutomatedBindings();
		autoChooser = AutoBuilder.buildAutoChooser("Center - Score L1A"); // Default auto program to run
		SmartDashboard.putData("Auto Mode", autoChooser);
    }
	/** called in robot teleop and auto initialize methods */
	public void initialize() {
        scoringSubsystem.initialize();
		ledLights.lightMode(robotMode);
	}
	/** proper deadband application for smooth control */
	private static double applyDeadband(double value){
		return applyDeadband(value, XBOX_DEADBAND);
	}
	private static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        // Rescale so the output goes from 0 to 1 outside the deadband
        double sign = Math.signum(value);
        double adjusted = (Math.abs(value) - deadband) / (1.0 - deadband);
        return sign * adjusted;
    }

	/** conditions the input axis with deadband and turbo 
	 * @param turboAxis assumes the turbo axis is 0 to 1
	*/
	private double conditionInput(double inputAxis, double turboAxis, double maxRange){
		//map the input which ranges from -1 to 1 and the turbo axis to a range of +/- MAX_CONTROL_SPEED
		inputAxis = applyDeadband(inputAxis);
		double turbo = turboAxis * (1 - 1/TURBO_MULTIPLE) + 1/TURBO_MULTIPLE ; //ranges from 1/turbo multiple to 1 with turbo axis from 0 to 1
		return inputAxis * maxRange * turbo;
	}

	public void printPose(){
		Pose2d test = drivetrain.getState().Pose;
		System.out.println("x " + test.getX());
		System.out.println("y " + test.getY());
		System.out.println("rot " + test.getRotation());
	}

	/**
	 * Configure only the drive to enable or disable
	 * @param enableDriving true to enable driving, false to disable
	 */
	private void configureDriveBindings(boolean enableDriving){
		if (enableDriving){
			// Drive Controls
			drivetrain.setDefaultCommand(
				// Drivetrain will execute this command periodically
				drivetrain.applyRequest(() ->
					drive.withVelocityX(conditionInput(-driverController.getLeftY(), driverController.getRightTriggerAxis() , MAX_CONTROL_SPEED)) // Drive forward with negative Y (forward)
						.withVelocityY(conditionInput(-driverController.getLeftX(), driverController.getRightTriggerAxis() , MAX_CONTROL_SPEED)) // Drive left with negative X (left)
						.withRotationalRate(conditionInput(-driverController.getRightX(), driverController.getRightTriggerAxis() , MAX_ANGULAR_RATE)) // Drive counterclockwise with negative X (left)
				)
			);
			// Reset the field-centric heading on left bumper press
			driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
		} else {
			drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            	drivetrain.applyRequest(() ->
					drive.withVelocityX(0 ) // no drive
						.withVelocityY(0) // no drive
						.withRotationalRate(0) // no turn
				)
        	);
		}
		// Brake Mode - Stop robot from being moved
		//driverController.x().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));
	}

	/** Created only to reduce Merge Conflicts while both working on this file */
	private void configureOperatorBindingsColin() {
		//// ------------------ Drivetrain Controls ------------------
		driverController.start().onTrue(Commands.runOnce(() -> toggleHangingMode())); // View button
        //// ----------------- Hanging Controls -----------------
		driverController.povUp().and(()-> robotMode == RobotModes.HangingMode).onTrue(hanger.extend());
        driverController.povDown().and(()-> robotMode == RobotModes.HangingMode).onTrue(hanger.retract());
		driverController.leftBumper().and(()-> robotMode == RobotModes.HangingMode).onTrue(hanger.intake());
        driverController.leftBumper().and(()-> robotMode == RobotModes.HangingMode).onFalse(hanger.stop());
        driverController.back().and(()-> robotMode == RobotModes.HangingMode).onTrue(hanger.manualRetract());
        driverController.back().and(()-> robotMode == RobotModes.HangingMode).onFalse(hanger.resetWinch());
        
        //// -------------------- Cancel All --------------------
        // operatorController.button(12).onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll())

        //// ---------------- General Use Commands ----------------
		 operatorController.button(8).onTrue(Commands.runOnce(()-> toggleAlgaeCoralMode()));
		 operatorController.button(7).onTrue(Commands.runOnce(()-> toggleManualMode())); // Back Button
		 operatorController.button(10).whileTrue(Commands.defer(()-> new WristFlipCommand(scoringSubsystem), Set.of(scoringSubsystem))); // Right Bumper
		 operatorController.pov(180).onTrue(Commands.defer(()->scoringSubsystem.moveArm(Position.HOME_FOR_CLIMB), Set.of(scoringSubsystem)));
		 scoringSubsystem.setDefaultCommand(new ManualScoringControlCommand(this, scoringSubsystem,
		 () -> applyDeadband(-operatorController.getRawAxis(1)),
		 () -> applyDeadband(-operatorController.getRawAxis(5)),
		 () -> applyDeadband(operatorController.getRawAxis(4))));

        //// --------------- Coral Handling Commands ---------------
		operatorController.axisGreaterThan(2,.1).and(()-> robotMode == RobotModes.CoralMode).onTrue(Commands.runOnce(() -> intakeSubsystem.coralIntake()))
			.onFalse(intakeSubsystem.runOnce(() -> intakeSubsystem.coralStop()));  // Left Trigger	
		operatorController.axisGreaterThan(3,.1).and(()-> robotMode == RobotModes.CoralMode).onTrue(Commands.runOnce(() -> intakeSubsystem.coralOuttake()))
			.onFalse(intakeSubsystem.runOnce(() -> intakeSubsystem.coralStop())); // Right Trigger	
		// operatorController.axisGreaterThan(2,.1).onFalse(intakeSubsystem.runOnce(() -> intakeSubsystem.coralStop())).and(()-> robotMode == RobotModes.CoralMode);   // Left Trigger	
		// operatorController.axisGreaterThan(3,.1).onFalse(intakeSubsystem.runOnce(() -> intakeSubsystem.coralStop())).and(()-> robotMode == RobotModes.CoralMode);   // Right Trigger	
		operatorController.button(5).and(()-> robotMode == RobotModes.CoralMode).onTrue(Commands.defer(()->scoringSubsystem.moveArm(Position.CORAL_GROUND_INTAKE), Set.of(scoringSubsystem))); // Left Bumper
		operatorController.button(1).and(()-> robotMode == RobotModes.CoralMode).onTrue(Commands.defer(()->scoringSubsystem.moveArm(Position.SCORE_L1), Set.of(scoringSubsystem))); 
		operatorController.button(2).and(()-> robotMode == RobotModes.CoralMode).onTrue(Commands.defer(()->scoringSubsystem.moveArm(Position.SCORE_L2), Set.of(scoringSubsystem)));
		operatorController.button(3).and(()-> robotMode == RobotModes.CoralMode).onTrue(Commands.defer(()->scoringSubsystem.moveArm(Position.SCORE_L3), Set.of(scoringSubsystem)));
		operatorController.button(4).and(()-> robotMode == RobotModes.CoralMode).onTrue(Commands.defer(()->scoringSubsystem.moveArm(Position.SCORE_L4), Set.of(scoringSubsystem)));
		operatorController.pov(0).and(()-> robotMode == RobotModes.CoralMode).onTrue(Commands.defer(()->scoringSubsystem.moveArm(Position.DRIVE_WITH_CORAL), Set.of(scoringSubsystem))); // Down on D-Pad
		operatorController.button(6).and(()-> robotMode == RobotModes.CoralMode).onTrue(Commands.defer(()->scoringSubsystem.PlaceCoralCommand(Position.SCORE_L4, intakeSubsystem), Set.of(scoringSubsystem))); // Right Bumper



		//// ----------------- Algae Handling Commands ----------------
 		operatorController.axisGreaterThan(2,.1).and(()-> robotMode == RobotModes.AlgaeMode).onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.algaeIntake()))
			.onFalse(intakeSubsystem.runOnce(() -> intakeSubsystem.algaeStop()));  // Left Trigger	
		operatorController.axisGreaterThan(3,.1).and(()-> robotMode == RobotModes.AlgaeMode).onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.algaeOuttake()))
			.onFalse(intakeSubsystem.runOnce(() -> intakeSubsystem.algaeStop())); // Right Trigger	
		new Trigger(()->robotMode == RobotModes.AlgaeMode).onFalse(intakeSubsystem.runOnce(() -> intakeSubsystem.coralStop()));
		// operatorController.axisGreaterThan(2,.1).onFalse(intakeSubsystem.runOnce(() -> intakeSubsystem.algaeStop())).and(()-> AlgaeMode);   // Left Trigger	
		// operatorController.axisGreaterThan(3,.1).onFalse(intakeSubsystem.runOnce(() -> intakeSubsystem.algaeStop())).and(()-> AlgaeMode);   // Right Trigger	
		operatorController.button(5).and(()-> robotMode == RobotModes.AlgaeMode).onTrue(Commands.defer(()->scoringSubsystem.moveArm(Position.ALGAE_GROUND_INTAKE), Set.of(scoringSubsystem)));
		operatorController.button(6).and(()-> robotMode == RobotModes.AlgaeMode).onTrue(Commands.defer(()->scoringSubsystem.moveArm(Position.ALGAE_LOLLIPOP_INTAKE), Set.of(scoringSubsystem))); 
		operatorController.pov(270).and(()-> robotMode == RobotModes.AlgaeMode).onTrue(Commands.defer(()->scoringSubsystem.moveArm(Position.SCORE_PROCESSOR), Set.of(scoringSubsystem)));
		operatorController.pov(0).and(()-> robotMode == RobotModes.AlgaeMode).onTrue(Commands.defer(()->scoringSubsystem.moveArm(Position.DRIVE_WITH_ALGAE), Set.of(scoringSubsystem)));
		operatorController.button(10).and(()-> robotMode == RobotModes.AlgaeMode).onTrue(Commands.defer(()->scoringSubsystem.DealgaeCommand(true, intakeSubsystem), Set.of(scoringSubsystem)));
		operatorController.button(9).and(()-> robotMode == RobotModes.AlgaeMode).onTrue(Commands.defer(()->scoringSubsystem.DealgaeCommand(false, intakeSubsystem), Set.of(scoringSubsystem)));

		//// -------- Manual Override + Encoder Reset --------
		// If Manual Override is false, become true
		// If Manual Override is true, reset encoder positions, and then become false
        // operatorController.button(7).onTrue(Commands.runOnce(() -> 
		// 	new ConditionalCommand(
		// 		new ParallelCommandGroup(
		// 			Commands.runOnce(() -> ElevatorSubsystem.resetEncoder()),
		// 			Commands.runOnce(() -> DifferentialSubsystem.resetEncoder()),
		// 			Commands.runOnce(() -> {robotMode = RobotModes.CoralMode;
		// 				ledLights.lightMode(robotMode);})
		// 		),				 
		// 		Commands.runOnce(() -> {robotMode = RobotModes.ManualMoveMode; ledLights.lightMode(RobotModes.ManualMoveMode);}),
		// 		() -> false)//robotMode == RobotModes.ManualMoveMode)
		// 	));
	}
	/** Created only to reduce Merge Conflicts while both working on this file */
	private void configureOperatorBindingsBrandon() {
		// button 2 is B on xbox controller
		//driverController.button(2).onTrue(Commands.defer(()->scoringSubsystem.moveArm(Position.SCORE_L1), Set.of(scoringSubsystem)));
		// button 9 is left joystick button on xbox controller
		//driverController.button(9).onTrue(Commands.defer(()->scoringSubsystem.moveArm(Position.HOME_FOR_CLIMB), Set.of(scoringSubsystem))); 

	}
	/** Created to reduce Merge Conflicts while both working on this file, and it also is a convenient place to store allt he auto scoring elements */
	private void configureAutomatedBindings(){
		//Testing Only area TODO comment out when not testing
		Pose2d frontofABlueRobotPose = new Pose2d(5.76,4.0, new Rotation2d(Units.degreesToRadians(90)));
		Pose2d frontofCRedRobotPose = new Pose2d(13.673,5.084, new Rotation2d(Units.degreesToRadians(150)));
		Pose2d frontofARedRobotPose = new Pose2d(11.7,4.0, new Rotation2d(Units.degreesToRadians(-90)));
		double backupDistance = 0.5; //meters to start away from reef
		Translation2d offset = new Translation2d(backupDistance,0);
		// Transform2d transform = new Transform2d(offset, new Rotation2d(0));
		Pose2d startPose = new Pose2d(frontofABlueRobotPose.getTranslation().plus(offset), new Rotation2d(Units.degreesToRadians(180)));
		Pose2d endPose = new Pose2d(frontofABlueRobotPose.getTranslation(), new Rotation2d(Units.degreesToRadians(180)));

		drivetrain.resetPose(frontofABlueRobotPose); //near center of field facing towards drivers
		//drivetrain.seedFieldCentric();
		PathConstraints constraints = new PathConstraints(
			1, 0.1,
			Units.degreesToRadians(270), Units.degreesToRadians(360));
		PathPlannerPath testPath2 =
                  new PathPlannerPath(
                      PathPlannerPath.waypointsFromPoses(
						startPose , endPose),
                      constraints,
                      new IdealStartingState(
                          0,
                          new Rotation2d(Units.degreesToRadians(90))),
                      new GoalEndState(0, new Rotation2d(Units.degreesToRadians(90))));
		PathPlannerPath testPath;
		try {
			testPath = PathPlannerPath.fromPathFile("A1");
		} catch (Exception e) {
			e.printStackTrace();
			throw (new RuntimeException("Loaded a path that does not exist."));
		}

		AutoScoringPathBuilder pathBuilder = new AutoScoringPathBuilder(true);
		driverController.button(2).onTrue(pathBuilder.goToScoringPosition(AutoScoringPathBuilder.C_L)); // B button
		// operatorController.button(1).whileTrue(PathFinderHelperCommands.followRelativePathCommand(new Pose2d(.5,0,new Rotation2d(0)), constraints, drivetrain)); // 
		//driverController.button(2).whileTrue(AutoBuilder.pathfindToPose(frontofABlueRobotPose, constraints)); // 
		driverController.button(1).whileTrue(AutoBuilder.pathfindThenFollowPath(testPath2, constraints)); // A button
		driverController.button(8).onTrue(Commands.runOnce(()-> drivetrain.resetPose(frontofCRedRobotPose))); // menu button
		//driverController.button(4).onTrue(Commands.runOnce(()-> drivetrain.resetPose(frontofCRedRobotPose))); // y button

		if(false){
				// Choosing where to score on Custom Controller
			CustomController.bt1().onTrue(new RunCommand(() -> {
				hexSide = AutoPlace.HexSide.A;
				side = AutoPlace.Side.one;
			}));
			CustomController.bt2().onTrue(new RunCommand(() -> {
				hexSide = AutoPlace.HexSide.A;
				side = AutoPlace.Side.two;
			}));
			CustomController.bt3().onTrue(new RunCommand(() -> {
				hexSide = AutoPlace.HexSide.B;
				side = AutoPlace.Side.one;
			}));
			CustomController.bt4().onTrue(new RunCommand(() -> {
				hexSide = AutoPlace.HexSide.B;
				side = AutoPlace.Side.two;
			}));
			CustomController.bt5().onTrue(new RunCommand(() -> {
				hexSide = AutoPlace.HexSide.C;
				side = AutoPlace.Side.one;
			}));
			CustomController.bt6().onTrue(new RunCommand(() -> {
				hexSide = AutoPlace.HexSide.C;
				side = AutoPlace.Side.two;
			}));
			CustomController.bt7().onTrue(new RunCommand(() -> {
				hexSide = AutoPlace.HexSide.D;
				side = AutoPlace.Side.one;
			}));
			CustomController.bt8().onTrue(new RunCommand(() -> {
				hexSide = AutoPlace.HexSide.D;
				side = AutoPlace.Side.two;
			}));
			CustomController.bt9().onTrue(new RunCommand(() -> {
				hexSide = AutoPlace.HexSide.E;
				side = AutoPlace.Side.one;
			}));
			CustomController.bt10().onTrue(new RunCommand(() -> {
				hexSide = AutoPlace.HexSide.E;
				side = AutoPlace.Side.two;
			}));
			CustomController.bt11().onTrue(new RunCommand(() -> {
				hexSide = AutoPlace.HexSide.F;
				side = AutoPlace.Side.one;
			}));
			CustomController.bt12().onTrue(new RunCommand(() -> {
				hexSide = AutoPlace.HexSide.F;
				side = AutoPlace.Side.two;
			}));
			CustomController.bt16().onTrue(new RunCommand(() -> {
				autoScoringPosition = Position.SCORE_L1;
			}));
			CustomController.bt17().onTrue(new RunCommand(() -> {
				autoScoringPosition = Position.SCORE_L2;
			}));
			CustomController.bt18().onTrue(new RunCommand(() -> {
				autoScoringPosition = Position.SCORE_L3;
			}));
			CustomController.bt19().onTrue(new RunCommand(() -> {
				autoScoringPosition = Position.SCORE_L4;
			}));


			//TODO remove this overide once testing completed
			hexSide = AutoPlace.HexSide.C;
			side = AutoPlace.Side.one;
			autoScoringPosition = Position.SCORE_L1;

			/// Autoplace command (Allow operator to also place)
			driverController.back().whileTrue(new AutoPlace(drivetrain, scoringSubsystem, new Node(autoScoringPosition, hexSide, side)));


		}
		
		
		// // Auto pickup command
		// // If wanting to pickup to score for level 1, press A, otherwise press Y
		// operatorController.y().whileTrue(new RunCommand(() -> level1Pickup = false));
		// operatorController.a().whileTrue(new RunCommand(() -> level1Pickup = true));
		// operatorController.leftBumper().whileTrue(new AutoPickup(drivetrain,
		// elevatorSubsystem,
		// () -> AutoPickup.getCoralSide(drivetrain.getState().Pose), level1Pickup));

	}
	
	public Command toggleAlgaeCoralMode() {
		if (robotMode == RobotModes.CoralMode) {
			robotMode = RobotModes.AlgaeMode;
		} else robotMode = RobotModes.CoralMode;

		ledLights.lightMode(robotMode);

		return new InstantCommand();
	}

	public Command toggleHangingMode() {
		if (robotMode == RobotModes.HangingMode) {
			robotMode = RobotModes.CoralMode;
		} else robotMode = RobotModes.HangingMode;

		ledLights.lightMode(robotMode);

		return new InstantCommand();
	}
	public Command toggleManualMode() {
		if (robotMode == RobotModes.ManualMoveMode) {
			robotMode = RobotModes.CoralMode;
			ElevatorSubsystem.resetEncoder();
			DifferentialSubsystem.resetEncoder();
		} else robotMode = RobotModes.ManualMoveMode;

		ledLights.lightMode(robotMode);

		return new InstantCommand();
	}

	public RobotModes getRobotMode() {
		return robotMode;
	}
	public Command getAutonomousCommand() {
		/* Run the path selected from the auto chooser */
		return autoChooser.getSelected();
	}

	private void registerCommands() {
		// Register the commands here

		//TODO move the positions into the subsystems and make the commands more simple by calling only one position 
		// NamedCommands.registerCommand("ArmToLevel1", new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, 
        //         ElevatorSubsystem.LEVEL1_POSITION, ElbowSubsystem.LOW_POS, ElbowSubsystem.START_POS_ELEVATION, 
        //         ElbowSubsystem.START_POS_ROTATION, 0.0));
        // NamedCommands.registerCommand("ArmHomingAfterLevel1", new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, 
        //         elevatorSubsystem.getPosition(), ElbowSubsystem.HOMING_POS, elbowSubsystem.getElevationPos(), 
        //         elbowSubsystem.getRotationPos(), ElevatorSubsystem.LEVEL1_POSITION));
        // NamedCommands.registerCommand("ArmToLevel2", new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, 
        //         ElevatorSubsystem.LEVEL2_POSITION, ElbowSubsystem.MIDS_POS, ElbowSubsystem.START_POS_ELEVATION, 
        //         ElbowSubsystem.START_POS_ROTATION, ElevatorSubsystem.LOW_POSITION));
        // NamedCommands.registerCommand("ArmToLevel3", new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, 
        //         ElevatorSubsystem.LEVEL3_POSITION, ElbowSubsystem.MIDS_POS, ElbowSubsystem.START_POS_ELEVATION, 
        //         ElbowSubsystem.START_POS_ROTATION, ElevatorSubsystem.LOW_POSITION));
        // NamedCommands.registerCommand("ArmToLevel4", new ArmToPosCommand(elevatorSubsystem, elbowSubsystem, 
        //         ElevatorSubsystem.LEVEL4_POSITION, ElbowSubsystem.HIGH_POS, ElbowSubsystem.START_POS_ELEVATION, 
        //         ElbowSubsystem.START_POS_ROTATION, ElevatorSubsystem.LOW_POSITION));
        NamedCommands.registerCommand("CoralIntake", intakeSubsystem.runOnce(() -> intakeSubsystem.coralIntake()));
        NamedCommands.registerCommand("CoralOutake", intakeSubsystem.runOnce(() -> intakeSubsystem.coralOuttake()));
        NamedCommands.registerCommand("IntakeStop", intakeSubsystem.runOnce(() -> intakeSubsystem.coralStop()));
	}
}