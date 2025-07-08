package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class TurnToAprilTagCommand extends Command {
    private final LimelightSubsystem limelight;
    private final SwerveSubsystem swerve;

    public TurnToAprilTagCommand(LimelightSubsystem limelight, SwerveSubsystem swerve) {
        this.limelight = limelight;
        this.swerve = swerve;
        addRequirements(limelight, swerve);
    }
}