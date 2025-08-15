package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmMotorSubsystem extends SubsystemBase {

    protected SparkMax armMotor = new SparkMax(11, MotorType.kBrushless);
    protected SparkMaxConfig defaultConfig = new SparkMaxConfig();
    private Timer timer = new Timer();
    public static int kSlot = 0;

    private double simulatedPosition = 0.0;
    private double simulatedVelocity = 0.0;
    private double targetKVoltage = 0.0;

    public ArmMotorSubsystem() {
        defaultConfig.smartCurrentLimit(30, 10, 100);
        defaultConfig.closedLoop.pid(1, 0, 0, ClosedLoopSlot.kSlot0);

        armMotor.configure(defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        timer.start();
    }

    public void applyVoltage(){
        armMotor.getClosedLoopController().setReference(targetKVoltage, ControlType.kVoltage);
    }

    public void increaseVoltage() {
        targetKVoltage += 0.1;
        applyVoltage();
    }

    public void decreaseVoltage() {
        targetKVoltage -= 0.1;
        applyVoltage();
    }

    public void driveMotorToPos(double position, int slot) {
        ClosedLoopSlot loopSlot = slotFromInt(slot);
        if (loopSlot != null) {
            armMotor.getClosedLoopController().setReference(position, ControlType.kPosition, loopSlot);
        } else {
            System.err.println("Invalid PID slot: " + slot);
        }
    }

    private ClosedLoopSlot slotFromInt(int slot) {
        if (slot == 0) return ClosedLoopSlot.kSlot0;
        return null;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", armMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm Velocity", armMotor.getEncoder().getVelocity());
    }

    @Override
    public void simulationPeriodic() {
        double appliedVolts = armMotor.getAppliedOutput() * 12.0;
        simulatedVelocity = appliedVolts * 0.1;
        simulatedPosition += simulatedVelocity * 0.02; // 20 ms loop

        armMotor.getEncoder().setPosition(simulatedPosition);

        SmartDashboard.putNumber("Sim Arm Position", simulatedPosition);
        SmartDashboard.putNumber("Sim Arm Velocity", simulatedVelocity);
    }
}
