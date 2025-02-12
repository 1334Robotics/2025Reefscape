package frc.robot.commands.laser;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.laser.LaserCanSubsystem;

public class MonitorLaserCanCommand extends Command {
    private final LaserCanSubsystem laserCanSubsystem;

    public MonitorLaserCanCommand(LaserCanSubsystem subsystem) {
        this.laserCanSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if (laserCanSubsystem.isValidMeasurement()) {
            System.out.printf("Target distance: %.1f mm%n", laserCanSubsystem.getDistance());
        } else {
            System.out.println("No valid measurement available");
        }
    }

    @Override
    public boolean isFinished() {
        return false;  // Run continuously until interrupted
    }
}