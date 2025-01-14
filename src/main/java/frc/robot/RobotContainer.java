package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;  // Import SmartDashboard
import edu.wpi.first.wpilibj.smartdashboard.Field2d;  // Import Field2d

public class RobotContainer {
    // Subsystems
    private final SwerveSubsystem swerve = new SwerveSubsystem();
    
    // Controllers
    private final CommandXboxController driverController = 
        new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);

    // Field2d object to visualize robot on the field
    private final Field2d field = new Field2d();

    public RobotContainer() {
        configureBindings();
        configureDefaultCommands();

        // Add the field to the SmartDashboard so we can view it
        SmartDashboard.putData("Field", field);
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(
            new SwerveDriveCommand(
                swerve,
                driverController.getHID(),
                () -> true  // Field relative by default
            )
        );
    }

    private void configureBindings() {
        // Reset gyro with Y button
        driverController.y().onTrue(Commands.runOnce(swerve::zeroHeading));
        
        // Lock wheels in X pattern with B button
        driverController.b().whileTrue(Commands.run(
            () -> swerve.drive(0, 0, 0, true),
            swerve
        ));
        
        // Toggle field relative/robot relative with X button
        driverController.x().onTrue(Commands.runOnce(
            () -> swerve.toggleFieldRelative()
        ));
        
        // Optional: Slow mode while holding right bumper
        driverController.rightBumper().whileTrue(
            new SwerveDriveCommand(
                swerve,
                driverController.getHID(),
                () -> true
            ).beforeStarting(() -> swerve.setMaxSpeed(0.5))  // 50% speed
             .finallyDo(() -> swerve.setMaxSpeed(1.0))       // Return to full speed
        );
    }

    public Command getAutonomousCommand() {
        // Add your autonomous command here
        return Commands.none();
    }

    // Method to update Field2d with robot's position
    public void updateField() {
        // Update the field with the robot's position (e.g., from swerve subsystem)
        field.setRobotPose(swerve.getPose());  // Assuming `getPose()` returns the robot's pose
    }
}
