package frc.robot.commands.laser;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.laser.LaserCanSubsystem;

public class MonitorLaserCanCommand extends Command {
    private final LaserCanSubsystem laserCanSubsystem;
    
    public MonitorLaserCanCommand() {
        this.laserCanSubsystem = RobotContainer.laserCanSubsystem;
        addRequirements(RobotContainer.laserCanSubsystem);
    }

    @Override
    public void execute() {
        if (laserCanSubsystem.isValidMeasurement()) {
            // Using %n instead of \n for platform-independent line separator
            // %n automatically uses the correct line ending for the current platform
            // (CRLF on Windows, LF on Unix/Linux/Mac)
            //System.out.printf("Target distance: %.1f mm%n", laserCanSubsystem.getDistance());
        } else {
            //System.out.println("No valid measurement available");
        }
    }

    @Override
    public boolean isFinished() {
        return false;  // Run continuously until interrupted
    }
}