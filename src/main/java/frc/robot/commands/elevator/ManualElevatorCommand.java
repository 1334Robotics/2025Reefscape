package frc.robot.commands.elevator;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ManualElevatorCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final Supplier<Integer> povSupplier;
    private static final double MANUAL_POWER = 0.2; // Reduced power for testing

    public ManualElevatorCommand(ElevatorSubsystem elevatorSubsystem, Supplier<Integer> povSupplier) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.povSupplier = povSupplier;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        int pov = povSupplier.get();
        double power = 0;
        
        // POV returns -1 when not pressed
        if (pov != -1) {
            if (pov == 0) { // UP
                power = MANUAL_POWER;
            } else if (pov == 180) { // DOWN
                power = -MANUAL_POWER;
            }
        }

        // Debug output
        SmartDashboard.putNumber("[ELEVATOR] POV Value", pov);
        SmartDashboard.putNumber("[ELEVATOR] Command Power", power);
        
        elevatorSubsystem.setManualControl(power);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setManualControl(0);
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
