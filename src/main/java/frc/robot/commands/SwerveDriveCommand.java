package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;

public class SwerveDriveCommand extends Command {
    private final SwerveSubsystem swerve;
    private final XboxController controller;
    private final BooleanSupplier fieldRelative;

    public SwerveDriveCommand(SwerveSubsystem swerve, XboxController controller, BooleanSupplier fieldRelative) {
        this.swerve = swerve;
        this.controller = controller;
        this.fieldRelative = fieldRelative;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        // Apply deadband to prevent drift
        double xSpeed = applyDeadband(-controller.getRawAxis(OIConstants.LEFT_Y_AXIS));
        double ySpeed = applyDeadband(-controller.getRawAxis(OIConstants.LEFT_X_AXIS));
        double rotation = applyDeadband(-controller.getRawAxis(OIConstants.RIGHT_X_AXIS));

        // Post raw values to SmartDashboard for debugging
        SmartDashboard.putNumber("Controller/LeftY", xSpeed);
        SmartDashboard.putNumber("Controller/LeftX", ySpeed);
        SmartDashboard.putNumber("Controller/RightX", rotation);
        SmartDashboard.putBoolean("Controller/FieldRelative", fieldRelative.getAsBoolean());

        // Drive the robot
        swerve.drive(xSpeed, ySpeed, rotation, fieldRelative.getAsBoolean());
    }

    private double applyDeadband(double value) {
        if (Math.abs(value) < OIConstants.CONTROLLER_DEADBAND) {
            return 0.0;
        }
        // Scale the output to go from 0 to 1 after deadband
        return Math.copySign((Math.abs(value) - OIConstants.CONTROLLER_DEADBAND) / (1 - OIConstants.CONTROLLER_DEADBAND), value);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        swerve.drive(0, 0, 0, true);
    }
}