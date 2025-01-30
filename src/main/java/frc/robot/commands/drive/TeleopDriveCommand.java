package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TeleopDriveCommand extends Command {
    private final SwerveSubsystem swerve;
    private final XboxController controller;
    
    public TeleopDriveCommand(SwerveSubsystem swerve, XboxController controller) {
        this.swerve = swerve;
        this.controller = controller;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        // Get joystick inputs (-1 to 1)
        double xSpeed = -controller.getLeftY();  // Forward/back
        double ySpeed = -controller.getLeftX();  // Left/right
        double rot = -controller.getRightX();    // Rotation
        
        // Dashboard - Raw Inputs
        SmartDashboard.putNumber("Teleop/Raw/X Input", xSpeed);
        SmartDashboard.putNumber("Teleop/Raw/Y Input", ySpeed);
        SmartDashboard.putNumber("Teleop/Raw/Rot Input", rot);
        
        // Apply deadband and scale inputs
        xSpeed = Math.abs(xSpeed) > 0.1 ? xSpeed * 2 : 0;
        ySpeed = Math.abs(ySpeed) > 0.1 ? ySpeed * 2 : 0;
        rot = Math.abs(rot) > 0.1 ? rot * 2 : 0;

        // Dashboard - Processed Inputs
        SmartDashboard.putNumber("Teleop/Processed/X Speed", xSpeed);
        SmartDashboard.putNumber("Teleop/Processed/Y Speed", ySpeed);
        SmartDashboard.putNumber("Teleop/Processed/Rot Speed", rot);

        // Dashboard - Robot State
        SmartDashboard.putBoolean("Teleop/Field Relative", true);
        SmartDashboard.putString("Teleop/Robot Pose", 
            String.format("X: %.2f, Y: %.2f, Rot: %.2f",
                swerve.getPose().getX(),
                swerve.getPose().getY(),
                swerve.getPose().getRotation().getDegrees()));

        // Drive robot
        swerve.drive(xSpeed, ySpeed, rot, true);
    }
}