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

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coral_detection_subsystem.CoralDetectionSubsystem;
import frc.robot.subsystems.elbow_subsystem.ElbowSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    PortForwarder.add(5801, "172.29.0.1", 5801);
    PortForwarder.add(5802, "172.29.0.1", 5802);
    PortForwarder.add(5803, "172.29.0.1", 5803);
    PortForwarder.add(5804, "172.29.0.1", 5804);
    PortForwarder.add(5805, "172.29.0.1", 5805);
    PortForwarder.add(5806, "172.29.0.1", 5806);
    PortForwarder.add(5807, "172.29.0.1", 5807);
    PortForwarder.add(5808, "172.29.0.1", 5808);
    PortForwarder.add(5809, "172.29.0.1", 5809);

    PortForwarder.add(5811, "172.29.1.1", 5801);
    PortForwarder.add(5812, "172.29.1.1", 5802);
    PortForwarder.add(5813, "172.29.1.1", 5803);
    PortForwarder.add(5814, "172.29.1.1", 5804);
    PortForwarder.add(5815, "172.29.1.1", 5805);
    PortForwarder.add(5816, "172.29.1.1", 5806);
    PortForwarder.add(5817, "172.29.1.1", 5807);
    PortForwarder.add(5818, "172.29.1.1", 5808);
    PortForwarder.add(5819, "172.29.1.1", 5809);

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

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    double txnc = CoralDetectionSubsystem.findCoralPos();
    if (txnc == Double.NaN || txnc == 0.0) {
      System.out.println("coral not found");
    } else{
      System.out.println("coral" + txnc);
    }
  }

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
