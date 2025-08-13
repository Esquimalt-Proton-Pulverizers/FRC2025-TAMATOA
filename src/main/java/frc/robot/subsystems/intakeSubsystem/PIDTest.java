package frc.robot.subsystems.intakeSubsystem;
import java.util.function.Supplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDTest extends SubsystemBase  {
    protected SparkMax inMotor = new SparkMax(4, MotorType.kBrushless);
    protected SparkMaxConfig baseConfig = new SparkMaxConfig();
    private Timer timer = new Timer();
    public static int kSlot = 0;
    private double targetPosition = 0.0;
    
    public PIDTest ()   {
        baseConfig.smartCurrentLimit(3,3,50);
        baseConfig.closedLoop.pid(1,0,0,ClosedLoopSlot.kSlot0);
        baseConfig.closedLoop.pid(1,0,0,ClosedLoopSlot.kSlot1);
        baseConfig.closedLoop.pid(1,0,0,ClosedLoopSlot.kSlot2);
        baseConfig.closedLoop.pid(1,0,0,ClosedLoopSlot.kSlot3);

        inMotor.configure(baseConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters);
        inMotor.getClosedLoopController().setReference(0, SparkMax.ControlType.kVoltage);
        timer.start();
    }
    public Command FollowStick(Supplier<Double> requestSupplier) {
        return new InstantCommand(() -> inMotor.getClosedLoopController().setReference(requestSupplier.get(), SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0));
    }
    public void toPos(double pos, int kSlot) {
        //swithc case for kslot
        switch (kSlot) {
            case 0:
                inMotor.getClosedLoopController().setReference(pos,SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
                break;
            case 1:
                inMotor.getClosedLoopController().setReference(pos,SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot1);
                break;
            case 2:
                inMotor.getClosedLoopController().setReference(pos,SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot2);
                break;
            case 3:
                inMotor.getClosedLoopController().setReference(pos,SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot3);
                break;
            default:
                System.out.println("Invalid kSlot value: " + kSlot);
        }
        targetPosition = pos;
        inMotor.getClosedLoopController().setReference(targetPosition, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    public Command pidCommand(double pos, int kSlot) {
        return new InstantCommand(() -> {
            toPos(pos, kSlot);
        });
    }
    

    
    @Override
    public void periodic() {
        if (timer.hasElapsed(2))    {

        System.out.println(inMotor.getEncoder().getPosition());
        System.out.println("KSlot = " + kSlot);
        timer.reset();
        }
    }
    public Command incrementStick(Supplier<Double> rpsSupplier, double speedMultiplier) {
        return new InstantCommand(() -> {
            double increment = rpsSupplier.get() / 50 * speedMultiplier;
            targetPosition += increment;
            inMotor.getClosedLoopController().setReference(targetPosition, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        },this); 
    };
    
}

