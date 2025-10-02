package frc.robot.subsystems.scoring_subsystem;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.scoring_subsystem.differential.DifferentialElevationRotationCommand;
import frc.robot.subsystems.scoring_subsystem.differential.DifferentialSubsystem;
import frc.robot.subsystems.scoring_subsystem.differential.SmartDifferentialElevationCommand;
import frc.robot.subsystems.scoring_subsystem.elevator.ElevatorSubsystem;
import frc.robot.subsystems.scoring_subsystem.elevator.ElevatorToPosCommand;

public class ManualMoveArmSubsystem extends SubsystemBase {
    private final DifferentialSubsystem differentialSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CommandGenericHID commandGenericHID = new CommandGenericHID(0); // Assuming joystick is on port 0
    private double speed; // Speed to move the subsystem in deg/sec or in/sec
    private boolean debugMode = true; // Set to true to enable debug prints
    private int subsystemToMove; // 0 for differential, 1 for wrist, 2 for elevator
    private double PERIOD = 0.02; // 20 ms
    private double currentTime = 0.0;
    double startPos;
    /**
     * 
     * @param subsystemToMove 0 for differential, 1 for wrist, 2 for elevator
     * @param buttonIsPressed
     * @param speed
     * @param differentialSubsystem
     * @param elevatorSubsystem
     */
    public ManualMoveArmSubsystem(int subsystemToMove/*, Supplier<Boolean> buttonIsPressed*/, double speed,DifferentialSubsystem differentialSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.differentialSubsystem = differentialSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        this.subsystemToMove = subsystemToMove;
        double startDifferentialElevation = differentialSubsystem.getElevationPos();
        currentTime = 0.0;
        if (subsystemToMove == 0) {
            double startPos = differentialSubsystem.getElevationPos();
        } else if (subsystemToMove == 1) {
            double startPos = differentialSubsystem.getRotationPos();
        } else if (subsystemToMove == 2) {
            double startPos = elevatorSubsystem.getPosition();
        } else {
            if (debugMode) {
                System.out.println("Invalid subsystemToMove value: " + subsystemToMove);
            }
        }
        
        
        if (debugMode) {
            System.out.println("ManualMoveArmSubsystem initialized with speed: " + speed + " and subsystemToMove: " + subsystemToMove);
        }
        // while (buttonIsPressed.equals(commandGenericHID.whileTrue())) {
            if (currentTime % PERIOD == 0) {
                if (debugMode) {
                    System.out.println("Current Time: " + currentTime);
                }
                if (subsystemToMove == 0) {
                    double position = startPos + speed/PERIOD * currentTime;
                    new SmartDifferentialElevationCommand(position, differentialSubsystem, elevatorSubsystem);
                    if (debugMode) {
                        System.out.println("Scheduling DifferentialElevationRotationCommand with speed: " + speed);
                    }
                } else if (subsystemToMove == 1) {
                    double position = startPos + speed/PERIOD * currentTime;
                    new DifferentialElevationRotationCommand(startDifferentialElevation, position, differentialSubsystem).schedule();
                    if (debugMode) {
                        System.out.println("Scheduling SmartElbowElevationCommand with speed: " + speed);
                    }
                } else if (subsystemToMove == 2) {
                    double position = startPos + speed/PERIOD * currentTime;
                    new ElevatorToPosCommand(position, elevatorSubsystem).schedule();
                    if (debugMode) {
                        System.out.println("Scheduling ElevatorToPosCommand with speed: " + speed);
                    }
                } else {
                    if (debugMode) {
                        System.out.println("Invalid subsystemToMove value: " + subsystemToMove);
                    }
                }
            }
        // }
    }
    @Override
    public void periodic() {
        currentTime += PERIOD;
    }



}
