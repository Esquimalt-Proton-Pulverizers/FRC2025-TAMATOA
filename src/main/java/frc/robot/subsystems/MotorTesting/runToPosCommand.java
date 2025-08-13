package frc.robot.subsystems.MotorTesting;

import edu.wpi.first.wpilibj2.command.Command;

public class runToPosCommand extends Command{
    private double positionTarget;

    public runToPosCommand(double pos){
        this.positionTarget =pos;

    }
    @Override
    public void execute(){
        System.out.println("");
    }
    
}
