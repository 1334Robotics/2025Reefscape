package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import swervelib.SwerveController;

public class DriveCommand extends Command {
    private final SwerveSubsystem swerve;  // Reference to drive system
    private final DoubleSupplier   vX;
    private final DoubleSupplier   vY;
    private final DoubleSupplier   omega;
    private final SwerveController controller;

    public DriveCommand(SwerveSubsystem swerveSubsystem, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega) {
        this.swerve = swerveSubsystem;     // Store reference
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.controller = RobotContainer.swerveSubsystem.getSwerveController();

        addRequirements(swerveSubsystem);   // Declare usage
    }

    @Override
    public void execute() {
        // This math is from previous years
        double xVelocity = Math.pow(this.vX.getAsDouble(), 3) * SwerveConstants.DRIVE_SPEED;
        double yVelocity = Math.pow(this.vY.getAsDouble(), 3) * SwerveConstants.DRIVE_SPEED;
        double angularVelocity = Math.pow(this.omega.getAsDouble(), 3);

        // Update the values within SmartDashboard
        SmartDashboard.putNumber("[DRIVE] X Velocity", xVelocity);
        SmartDashboard.putNumber("[DRIVE] Y Velocity", yVelocity);
        SmartDashboard.putNumber("[DRIVE] Angular Velocity", angularVelocity);

        // Use stored reference to control robot
        this.swerve.drive(new Translation2d(xVelocity * SwerveConstants.MAX_SPEED,
                                            yVelocity * SwerveConstants.MAX_SPEED),
                          angularVelocity * controller.config.maxAngularVelocity);
    }

    @Override
    public boolean isFinished() {
        // The drive command will never finish
        return false;
    }
}
