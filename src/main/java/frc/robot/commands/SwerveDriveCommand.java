package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
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
        double xSpeed = -controller.getLeftY() * DriveConstants.MAX_SPEED;
        double ySpeed = -controller.getLeftX() * DriveConstants.MAX_SPEED;
        double rotation = -controller.getRightX() * DriveConstants.MAX_ANGULAR_SPEED;
        
        swerve.drive(xSpeed, ySpeed, rotation, fieldRelative.getAsBoolean());
    }
}
