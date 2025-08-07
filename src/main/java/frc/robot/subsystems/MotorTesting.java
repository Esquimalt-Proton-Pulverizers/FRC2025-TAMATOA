package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;

public class MotorTesting extends SubsystemBase{
    
    protected SparkMax intakeMotor = new SparkMax(4, MotorType.kBrushless);
    protected SparkMaxConfig defaultConfig = new SparkMaxConfig();
    private Timer timer = new Timer();

    public MotorTesting(){
        defaultConfig.smartCurrentLimit(3,1,100);
        defaultConfig.closedLoop.pid(0.1,0,0,ClosedLoopSlot.kSlot0);
        defaultConfig.closedLoop.pid(1,0,0,ClosedLoopSlot.kSlot1);
        defaultConfig.closedLoop.pid(1,0,.1,ClosedLoopSlot.kSlot2);
        defaultConfig.closedLoop.pid(0,0,1,ClosedLoopSlot.kSlot3);

        intakeMotor.configure(defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        intakeMotor.getClosedLoopController().setReference(0, ControlType.kVoltage);
        timer.start();
    }
    public void driveMotorToPos(double position){
        intakeMotor.getClosedLoopController().setReference(position, ControlType.kPosition,ClosedLoopSlot.kSlot0);

    }
    
    @Override
    public void periodic() {
        if (timer.hasElapsed(2)){
            System.out.println(intakeMotor.getEncoder().getPosition());
            timer.reset();
        }
    }

}
