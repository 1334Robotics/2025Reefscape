package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import java.io.File;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private boolean isFieldRelative = true;
    private double speedMultiplier = 1.0;
    private Pose2d previousPose = new Pose2d();
    private Rotation2d previousRotation = new Rotation2d();
    private static final double MINIMUM_ROTATION_MAGNITUDE = 1e-3;
    
    // Simulation-specific constants
    private static final double ENCODER_COUNTS_PER_ROTATION = 2048.0;
    private static final double DRIVE_GEAR_RATIO = 6.75;
    private static final double ANGLE_GEAR_RATIO = 12.8;
    private double[] lastDrivePositions = new double[4];
    private double[] lastAnglePositions = new double[4];

    public SwerveSubsystem() {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        
        try {
            String directoryPath = RobotBase.isReal() ? 
                "/home/lvuser/deploy" : 
                "src/main/deploy";
                
            System.out.println("Loading swerve drive configuration from: " + directoryPath);
            
            if (RobotBase.isSimulation()) {
                System.out.println("Running in simulation mode - enabling simulation features");
                SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
                initializeSimulation();
            }
            
            SwerveParser parser = new SwerveParser(new File(directoryPath));
            swerveDrive = parser.createSwerveDrive(
                DriveConstants.MAX_SPEED_METERS_PER_SECOND,
                new Pose2d(0, 0, Rotation2d.fromDegrees(0))
            );
            
        } catch (Exception e) {
            throw new RuntimeException("Failed to load swerve drive configuration", e);
        }
    }

    /**
     * Drive the robot using the specified speeds.
     */
    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
        // Apply speed multiplier and deadband
        xSpeed = applyDeadband(xSpeed * speedMultiplier, DriveConstants.DEADBAND) * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
        ySpeed = applyDeadband(ySpeed * speedMultiplier, DriveConstants.DEADBAND) * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
        rotation = applyDeadband(rotation * speedMultiplier, DriveConstants.DEADBAND) * DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        // Get current heading and validate it
        Rotation2d currentHeading = validateRotation(getHeading());
            
        ChassisSpeeds chassisSpeeds = fieldRelative && isFieldRelative ?
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, currentHeading) :
            new ChassisSpeeds(xSpeed, ySpeed, rotation);
            
        swerveDrive.drive(chassisSpeeds);
    }

    /**
     * Validates the rotation value to prevent invalid readings.
     */
    private Rotation2d validateRotation(Rotation2d rotation) {
        double cos = rotation.getCos();
        double sin = rotation.getSin();
        
        if (Math.abs(cos) < MINIMUM_ROTATION_MAGNITUDE && Math.abs(sin) < MINIMUM_ROTATION_MAGNITUDE) {
            return previousRotation;
        }
        
        previousRotation = rotation;
        return rotation;
    }

    /**
     * Gets the current heading of the robot.
     */
    public Rotation2d getHeading() {
        if (RobotBase.isSimulation()) {
            Pose2d currentPose = swerveDrive.getPose();
            Translation2d movement = currentPose.getTranslation().minus(previousPose.getTranslation());
            
            if (movement.getNorm() > 0.0001) {
                previousRotation = Rotation2d.fromRadians(Math.atan2(movement.getY(), movement.getX()));
            }
            
            previousPose = currentPose;
            return previousRotation;
        }
        
        return swerveDrive.getYaw();
    }

    /**
     * Applies a deadband to the input value.
     */
    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return Math.copySign((Math.abs(value) - deadband) / (1.0 - deadband), value);
    }

    /**
     * Zeros the heading of the robot.
     */
    public void zeroHeading() {
        swerveDrive.zeroGyro();
        previousRotation = new Rotation2d();
    }

    /**
     * Toggles between field-relative and robot-relative control.
     */
    public void toggleFieldRelative() {
        isFieldRelative = !isFieldRelative;
        SmartDashboard.putBoolean("Field Relative", isFieldRelative);
    }

    /**
     * Sets the maximum speed multiplier.
     */
    public void setMaxSpeed(double multiplier) {
        this.speedMultiplier = multiplier;
        SmartDashboard.putNumber("Speed Multiplier", speedMultiplier);
    }

    private void initializeSimulation() {
        lastDrivePositions = new double[4];
        lastAnglePositions = new double[4];
        
        for (int i = 0; i < 4; i++) {
            String prefix = "Sim/Module" + i;
            SmartDashboard.putNumber(prefix + "/DrivePosition", 0);
            SmartDashboard.putNumber(prefix + "/AnglePosition", 0);
            SmartDashboard.putNumber(prefix + "/DriveVelocity", 0);
            SmartDashboard.putNumber(prefix + "/AngleVelocity", 0);
        }
    }

    @Override
public void simulationPeriodic() {
    if (!RobotBase.isSimulation()) return;

    var moduleStates = swerveDrive.getStates();
    
    // Update each module's simulation
    for (int i = 0; i < moduleStates.length; i++) {
        var state = moduleStates[i];
        
        // Calculate drive motor simulation values
        double driveVelocityRPS = state.speedMetersPerSecond / (Math.PI * 0.1016); // Convert m/s to RPS
        double drivePositionRotations = driveVelocityRPS * 0.02; // Position change in 20ms
        lastDrivePositions[i] += drivePositionRotations;
        
        // Calculate angle motor simulation values
        double targetAngleRotations = state.angle.getRotations();
        double angleVelocityRPS = (targetAngleRotations - lastAnglePositions[i]) / 0.02;
        lastAnglePositions[i] = targetAngleRotations;
        
        // Convert to encoder counts
        double driveEncoderCounts = lastDrivePositions[i] * DRIVE_GEAR_RATIO * ENCODER_COUNTS_PER_ROTATION;
        double angleEncoderCounts = lastAnglePositions[i] * ANGLE_GEAR_RATIO * ENCODER_COUNTS_PER_ROTATION;
        
        // Update SmartDashboard with simulated values
        String prefix = "Sim/Module" + i;
        SmartDashboard.putNumber(prefix + "/DrivePosition", driveEncoderCounts);
        SmartDashboard.putNumber(prefix + "/AnglePosition", angleEncoderCounts);
        SmartDashboard.putNumber(prefix + "/DriveVelocity", driveVelocityRPS * DRIVE_GEAR_RATIO * ENCODER_COUNTS_PER_ROTATION / 10.0);
        SmartDashboard.putNumber(prefix + "/AngleVelocity", angleVelocityRPS * ANGLE_GEAR_RATIO * ENCODER_COUNTS_PER_ROTATION / 10.0);
        
        // Update rotor sensor values
        SmartDashboard.putNumber("Talon FX (" + (i*2+1) + ")/Rotor Sensor/position", driveEncoderCounts);
        SmartDashboard.putNumber("Talon FX (" + (i*2+1) + ")/Rotor Sensor/velocity", driveVelocityRPS * ENCODER_COUNTS_PER_ROTATION);
        SmartDashboard.putNumber("Talon FX (" + (i*2+2) + ")/Rotor Sensor/position", angleEncoderCounts);
        SmartDashboard.putNumber("Talon FX (" + (i*2+2) + ")/Rotor Sensor/velocity", angleVelocityRPS * ENCODER_COUNTS_PER_ROTATION);
    }

    // Simulate battery voltage drop based on motor usage
    double totalCurrentDraw = 0;
    for (var state : moduleStates) {
        totalCurrentDraw += Math.abs(state.speedMetersPerSecond) * 2.0;
    }
    
    double batteryVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentDraw);
    RoboRioSim.setVInVoltage(batteryVoltage);
    
    swerveDrive.updateOdometry();
}
}
