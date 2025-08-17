package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.RobotBase;
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

    private double simulatedAppliedOutput = 0.0;

    public ArmMotorSubsystem() {
        defaultConfig.smartCurrentLimit(30, 10, 100);
        defaultConfig.closedLoop.pid(1, 0, 0, ClosedLoopSlot.kSlot0);

        armMotor.configure(defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        armMotor.getClosedLoopController().setReference(0, ControlType.kVoltage);

        timer.start();
    }

    //for sim
    public void setMotorVoltage(double volts) {
        targetKVoltage = volts;
    
        if (RobotBase.isSimulation()) {
            simulatedAppliedOutput = volts / 12.0;   
        } else {
            armMotor.getClosedLoopController().setReference(volts, ControlType.kVoltage);
        }
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

        if (timer.hasElapsed(2)){ 
            System.out.println(("target kVoltage: " + targetKVoltage)); 
            timer.reset();
        }
    }

    public double getSimAppliedOutput() {
        if (RobotBase.isSimulation()) {
        return simulatedAppliedOutput;
        } else {
        return armMotor.getAppliedOutput(); 
        }
    }

    @Override
    public void simulationPeriodic() {
    // Use sim-tracked output instead of real JNI
    double appliedVolts;
    if (RobotBase.isSimulation()) {
    appliedVolts = getSimAppliedOutput() * 12.0; // fake output
    } else {
    appliedVolts = armMotor.getAppliedOutput() * 12.0; // real output
    }

    // Update sim physics
    simulatedVelocity = appliedVolts * 0.1;          // simple gain factor
    simulatedPosition += simulatedVelocity * 0.02;   // 20 ms loop

    // Feed fake encoder values back
    armMotor.getEncoder().setPosition(simulatedPosition);

    // Dashboard feedback
    SmartDashboard.putNumber("Sim Arm Position", simulatedPosition);
    SmartDashboard.putNumber("Sim Arm Velocity", simulatedVelocity);
    }
}
