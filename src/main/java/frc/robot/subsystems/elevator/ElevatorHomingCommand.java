package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorHomingCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private Timer timer = new Timer();
    private boolean finished;
    private SparkMaxConfig homingConfig=new SparkMaxConfig();

    public ElevatorHomingCommand(ElevatorSubsystem elevatorSubsystem){
      this.addRequirements(elevatorSubsystem);
      this.elevatorSubsystem=elevatorSubsystem;
      
      finished=false;
      homingConfig.smartCurrentLimit(1,4,50);
      
    };
    @Override
    public void initialize() {
      // TODO Auto-generated method stub
      elevatorSubsystem.elevatorMotor.setVoltage(5);
      elevatorSubsystem.elevatorMotor.configure(homingConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      finished=false;
      timer.reset();
      timer.start();
    }
    @Override
    public void execute() {
      //if elevatorMotor.getOutputCurrent()
      if (timer.hasElapsed(.5)){
        
        if (elevatorSubsystem.elevatorEncoder.getVelocity()<300||timer.hasElapsed(5)){
          finished = true;
          elevatorSubsystem.elevatorMotor.setVoltage(0);
        }        
      }      
    }

    @Override
    public void end(boolean interrupted) {
      if (!interrupted){
        elevatorSubsystem.elevatorEncoder.setPosition(0);
        elevatorSubsystem.elevatorClosedLoopController.setReference(0, SparkMax.ControlType.kPosition);
      }
      elevatorSubsystem.elevatorMotor.configure(elevatorSubsystem.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

      
        
    }
    
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return finished;
    }
  
    
  }