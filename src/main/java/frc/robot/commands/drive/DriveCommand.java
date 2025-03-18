package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.drive.DriveController.Controller;
import swervelib.SwerveController;

public class DriveCommand extends Command {
    private final DoubleSupplier   vX;
    private final DoubleSupplier   vY;
    private final DoubleSupplier   omega;
    private final SwerveController controller;

    public DriveCommand(DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega) {
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.controller = RobotContainer.swerveSubsystem.getSwerveController();

        addRequirements(RobotContainer.swerveSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.driveController.requestControl(Controller.MANUAL);
    }

    @Override
    public void execute() {
        // This math is from previous years
        double xVelocity = Math.pow(this.vX.getAsDouble(), 3) * SwerveConstants.DRIVE_SPEED;
        double yVelocity = Math.pow(this.vY.getAsDouble(), 3) * SwerveConstants.DRIVE_SPEED;
        double angularVelocity = Math.pow(this.omega.getAsDouble(), 3) * SwerveConstants.DRIVE_SPEED;

        // Update the values within SmartDashboard (The config is off 90 degrees, so this is what needs to happen)
        SmartDashboard.putNumber("[DRIVE] X Velocity", -yVelocity);
        SmartDashboard.putNumber("[DRIVE] Y Velocity", -xVelocity);
        SmartDashboard.putNumber("[DRIVE] Angular Velocity", angularVelocity);

        // Drive
        RobotContainer.driveController.requestControl(Controller.MANUAL);
        RobotContainer.driveController.drive(Controller.MANUAL, new Translation2d(-yVelocity * SwerveConstants.MAX_SPEED,
                                                                                  -xVelocity * SwerveConstants.MAX_SPEED),
                                             angularVelocity * controller.config.maxAngularVelocity);
    }

    @Override
    public boolean isFinished() {
        // The drive command will never finish
        return false;
    }
}