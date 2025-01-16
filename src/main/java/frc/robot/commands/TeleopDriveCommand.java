package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopDriveCommand extends Command {
    private final SwerveSubsystem driveSubsystem;
    private final XboxController controller;

    public TeleopDriveCommand(SwerveSubsystem driveSubsystem, XboxController controller) {
        this.driveSubsystem = driveSubsystem;
        this.controller = controller;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double xSpeed = -controller.getLeftY();
        double ySpeed = -controller.getLeftX();
        double rotation = -controller.getRightX();
        
        // Added the missing boolean parameter for field-relative control
        driveSubsystem.drive(xSpeed, ySpeed, rotation, true);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}