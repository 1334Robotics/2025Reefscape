package frc.robot.commands.mailbox;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorLevel;

public class ShootCommand extends Command {
    private final Timer timer = new Timer();
    private boolean hasStartedShooting = false;
    private static final double SHOOT_TIMEOUT = 2.0; // Timeout after 2 seconds

    public ShootCommand() {
        addRequirements(RobotContainer.mailboxHandler);
        addRequirements(RobotContainer.mailboxSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ShootCommand: Starting shoot sequence");
        timer.reset();
        timer.start();
        hasStartedShooting = false;
        SmartDashboard.putString("Shoot Command Status", "Starting");
    }

    @Override
    public void execute() {
        // Only start shooting if we have a game piece detected
        if (RobotContainer.mailboxHandler.isGamePieceAtOutput()) {
            if (!hasStartedShooting) {
                System.out.println("ShootCommand: Game piece detected, initiating shoot");
                RobotContainer.mailboxHandler.allowShoot();
                hasStartedShooting = true;
                timer.reset(); // Reset timer when we actually start shooting
            }
        } else if (!hasStartedShooting) {
            SmartDashboard.putString("Shoot Command Status", "Waiting for game piece");
        }
    }

    @Override
    public boolean isFinished() {
        // Finish if:
        // 1. We started shooting and the game piece is no longer detected (successful shot)
        // 2. We've timed out
        if (hasStartedShooting && !RobotContainer.mailboxHandler.isGamePieceAtOutput()) {
            System.out.println("ShootCommand: Shot completed - game piece no longer detected");
            return true;
        }
        
        if (timer.hasElapsed(SHOOT_TIMEOUT)) {
            System.out.println("ShootCommand: Timed out after " + SHOOT_TIMEOUT + " seconds");
            return true;
        }
        
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.mailboxSubsystem.stop();
        timer.stop();
        
        if (interrupted) {
            System.out.println("ShootCommand: Interrupted!");
            SmartDashboard.putString("Shoot Command Status", "Interrupted");
        } else {
            System.out.println("ShootCommand: Completed");
            SmartDashboard.putString("Shoot Command Status", "Completed");
        }
    }
}
