package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;//NEW CAL
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveConstants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import java.io.File;
import java.io.IOException;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private boolean fieldRelative;
    private int count = 0; //NEW CAL
    
    public SwerveSubsystem() {
        this.fieldRelative = false;
        SmartDashboard.putBoolean("[SWERVE] Field Relative", this.fieldRelative);
        
        // Create the swerve drive
        File swerveDirectory = new File(Filesystem.getDeployDirectory(), SwerveConstants.SWERVE_DRIVE_DIRECTORY);
        try {
            this.swerveDrive = new SwerveParser(swerveDirectory).createSwerveDrive(SwerveConstants.MAX_SPEED);
        } catch(IOException e) {
            throw new RuntimeException(e);
        }
        
        // Turn off heading correction and cosine compensation
        this.swerveDrive.setHeadingCorrection(false);
        this.swerveDrive.setCosineCompensator(false);
    }

    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }
    
    public void drive(Translation2d translation, double rotation) {
        swerveDrive.drive(translation, rotation, this.fieldRelative, false);
    }

    public void zeroGyro() {
        this.swerveDrive.zeroGyro();
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }
    
    public void setFieldRelative(boolean fieldRelative) {
        this.fieldRelative = fieldRelative;
        SmartDashboard.putBoolean("[SWERVE] Field Relative", this.fieldRelative);
    }
    
    public boolean isFieldRelative() {
        return fieldRelative;
    }
//NEW CAL
    /**
     * Get the current chassis speeds of the robot
     * @return ChassisSpeeds object containing vx, vy, and omega
     */
    public ChassisSpeeds getChassisSpeeds() {
        return swerveDrive.getRobotVelocity();
    }
//NEW CAL
    /**
     * Get the current odometry pose
     * @return Current pose from odometry
     */
    public Pose2d getOdometryPose() {
        return swerveDrive.getPose();
    }
//NEW CAL
    /**
     * Drive the robot with given speeds
     * @param xSpeed Forward/backward speed (m/s)
     * @param ySpeed Left/right speed (m/s)
     * @param rot Angular speed (rad/s)
     * @param fieldRelative Whether speeds are field-relative
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        swerveDrive.drive(
            new Translation2d(xSpeed, ySpeed),
            rot,
            fieldRelative,
            false  // Open loop
        );
    }
//NEW CAL
    @Override
    public void periodic() {
        // Debug swerve state
        SmartDashboard.putNumber("Swerve/UpdateCount", count++);
        
        var pose = getPose();
        SmartDashboard.putString("Swerve/Pose", 
            String.format("X: %.2f, Y: %.2f, Rot: %.2f",
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees()));

        var speeds = getChassisSpeeds();
        SmartDashboard.putNumber("Swerve/VX", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Swerve/VY", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Omega", speeds.omegaRadiansPerSecond);

        // Debug module states
        var states = swerveDrive.getStates();
        for (int i = 0; i < states.length; i++) {
            SmartDashboard.putNumber("Swerve/Module" + i + "/Speed", states[i].speedMetersPerSecond);
            SmartDashboard.putNumber("Swerve/Module" + i + "/Angle", states[i].angle.getDegrees());
        }
    }
}