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
	public static int kSlot = 0;

    public MotorTesting(){
        defaultConfig.smartCurrentLimit(30,10,100);
        defaultConfig.closedLoop.pid(1,0,0,ClosedLoopSlot.kSlot0);
        defaultConfig.closedLoop.pid(10,0,0,ClosedLoopSlot.kSlot1);
        defaultConfig.closedLoop.pid(1,0,1,ClosedLoopSlot.kSlot2);
        defaultConfig.closedLoop.pid(0,0.00001,0,ClosedLoopSlot.kSlot3);

        intakeMotor.configure(defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        intakeMotor.getClosedLoopController().setReference(0, ControlType.kVoltage);
        timer.start();
    }

    public void driveMotorToPos(double position, int slot){
        ClosedLoopSlot loopSlot = slotFromInt(slot);
        if (loopSlot != null) {
            intakeMotor.getClosedLoopController().setReference(position, ControlType.kPosition, loopSlot);
        } else {
            System.err.println("Invalid PID slot: " + slot);
        }
    }

    public void kSlotIncrementOne(){
      if (kSlot < 3){
        kSlot++;
      }  
    }

    public void kSlotDecrementOne(){
       if (kSlot > 0){
        kSlot--;
       }
    }

    private ClosedLoopSlot slotFromInt(int slot) {
        switch (slot) {
            case 0: return ClosedLoopSlot.kSlot0;
            case 1: return ClosedLoopSlot.kSlot1;
            case 2: return ClosedLoopSlot.kSlot2;
            case 3: return ClosedLoopSlot.kSlot3;
            default: return null; 
        }
    }
    //     if (slot == 0){
    //          intakeMotor.getClosedLoopController().setReference(position, ControlType.kPosition,ClosedLoopSlot.kSlot0);
    //     } else if (slot == 1){
    //         intakeMotor.getClosedLoopController().setReference(position, ControlType.kPosition,ClosedLoopSlot.kSlot1);
    //     }else if (slot == 2){
    //         intakeMotor.getClosedLoopController().setReference(position, ControlType.kPosition,ClosedLoopSlot.kSlot2);
    //     }else if (slot == 3){
    //         intakeMotor.getClosedLoopController().setReference(position, ControlType.kPosition,ClosedLoopSlot.kSlot3);
    //     }

    // }
    
    @Override
    public void periodic() {
        if (timer.hasElapsed(2)){
            System.out.println(intakeMotor.getEncoder().getPosition());
            System.out.println("kSlot value: " + kSlot);
            timer.reset();
            
        }
    }

}
