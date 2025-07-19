// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm_subsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArmMoveAndFinishCommand extends Command {
  double elevation;
  double rotation; 
  private boolean atPosition = false;

  public ArmMoveAndFinishCommand(double elevation, double rotation,ArmSubsystem armSubsystem) {
    this.elevation = elevation;
    this.rotation = rotation;
    this.addRequirements(armSubsystem);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting Arm Move");
    
    atPosition = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (true){
      atPosition=true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atPosition;
  }
}