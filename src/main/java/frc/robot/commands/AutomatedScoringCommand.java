package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class AutomatedScoringCommand extends Command{
  
    public AutomatedScoringCommand() {

    }
    
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      System.out.println("Button pressed");
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
  }
  