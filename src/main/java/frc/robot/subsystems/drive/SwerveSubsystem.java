package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Paths;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private boolean isFieldRelative = true;
    private double speedMultiplier = 1.0;

    public SwerveSubsystem() {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        
        try {
            String directoryPath = "src/main/deploy";
            String filePath = directoryPath + "/swervedrive.json";
            System.out.println("Loading swerve drive configuration from: " + filePath);
            
            // Print the content of the JSON file for debugging
            String content = new String(Files.readAllBytes(Paths.get(filePath)));
            System.out.println("Content of swervedrive.json: " + content);
            
            SwerveParser parser = new SwerveParser(new File(directoryPath));
            swerveDrive = parser.createSwerveDrive(
                DriveConstants.MAX_SPEED, 
                new Pose2d(0, 0, new Rotation2d(1, 0))  // Ensure non-zero values for Rotation2d
            );
        } catch (Exception e) {
            throw new RuntimeException("Failed to load swerve drive configuration", e);
        }
    }

    public void toggleFieldRelative() {
        isFieldRelative = !isFieldRelative;
    }

    public void setMaxSpeed(double multiplier) {
        this.speedMultiplier = multiplier;
    }

    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
        xSpeed *= speedMultiplier;
        ySpeed *= speedMultiplier;
        rotation *= speedMultiplier;
        
        ChassisSpeeds chassisSpeeds = fieldRelative ?
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, getHeading()) :
            new ChassisSpeeds(xSpeed, ySpeed, rotation);
            
        swerveDrive.drive(chassisSpeeds);
    }

    public Rotation2d getHeading() {
        return swerveDrive.getOdometryHeading();
    }

    public void zeroHeading() {
        swerveDrive.zeroGyro();
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading().getDegrees());
        SmartDashboard.putString("Robot Pose", getPose().toString());
        SmartDashboard.putBoolean("Field Relative", isFieldRelative);
    }
}
