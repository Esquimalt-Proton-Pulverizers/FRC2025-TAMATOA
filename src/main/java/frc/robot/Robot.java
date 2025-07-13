// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.elbow_subsystem.ElbowSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    ElbowSubsystem.initialize();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    ElbowSubsystem.initialize();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}
  
  private SparkMax testMotor = new SparkMax(20, MotorType.kBrushless);
  private SparkMaxConfig testConfig = new SparkMaxConfig();
  private SparkClosedLoopController testController = testMotor.getClosedLoopController();

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    testConfig.smartCurrentLimit(1,8,50);

    testConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.1).i(0.000001).d(0.0000)
    .outputRange(-.2, .6, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
