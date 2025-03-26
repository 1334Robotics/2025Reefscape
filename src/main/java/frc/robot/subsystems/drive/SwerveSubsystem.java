package frc.robot.subsystems.drive;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.kinematics.ChassisSpeeds;//NEW CAL
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoConfigurer;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.SimulationConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.imu.SwerveIMU;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIOSim;
import edu.wpi.first.units.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import static edu.wpi.first.units.Units.Meter;
import edu.wpi.first.wpilibj2.command.Commands;
import com.pathplanner.lib.auto.NamedCommands;



public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private boolean fieldRelative;
    private SwerveDriveSimulation swerveDriveSimulation;
    private int count = 0;

    public SwerveSubsystem() {
        this.fieldRelative = false;
        final GyroIO gyroIO;
        final GyroSimulation gyroSimulation;
        final ModuleIO[] moduleIOs;
        SmartDashboard.putBoolean("[SWERVE] Field Relative", this.fieldRelative);
        
        // Get alliance dynamically instead of hardcoding
        boolean blueAlliance = true; // Default to blue if alliance cannot be determined
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            blueAlliance = alliance.get() == DriverStation.Alliance.Blue;
            SmartDashboard.putString("[SWERVE] Initial Alliance", blueAlliance ? "BLUE" : "RED");
        } else {
            SmartDashboard.putString("[SWERVE] Initial Alliance", "UNKNOWN (defaulting to BLUE)");
        }
        
        // Set the starting pose based on the alliance color
        Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(5.89),
                                                                          Meter.of(5.5)),
                                                        Rotation2d.fromDegrees(180))
                                           : new Pose2d(new Translation2d(Meter.of(8.05),
                                                                          Meter.of(5.5)),
                                                        Rotation2d.fromDegrees(0));
        
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        // Create the swerve drive
        File swerveDirectory = new File(Filesystem.getDeployDirectory(), SwerveConstants.SWERVE_DRIVE_DIRECTORY);
        try {
            this.swerveDrive = new SwerveParser(swerveDirectory).createSwerveDrive(SwerveConstants.MAX_SPEED, startingPose);


            if (Robot.isSimulation()) {
                // Create and configure a drivetrain simulation configuration
                final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
                        // Specify gyro type (for realistic gyro drifting and error simulation)
                        .withGyro(COTS.ofPigeon2())
                        // Specify swerve module (for realistic swerve dynamics)
                        .withSwerveModule(new SwerveModuleSimulationConfig(
                                DCMotor.getKrakenX60(SwerveConstants.NUM_DRIVE_MOTORS), // Drive motor is a Kraken X60
                                DCMotor.getKrakenX60(SwerveConstants.NUM_STEER_MOTORS), // Steer motor is a Kraken X60
                                SwerveConstants.DRIVE_GEAR_RATIO, // Drive motor gear ratio.
                                SwerveConstants.STEER_GEAR_RATIO, // Steer motor gear ratio.
                                SimulationConstants.DRIVE_FRICTION_VOLTAGE, // Drive friction voltage.
                                SimulationConstants.STEER_FRICTION_VOLTAGE, // Steer friction voltage
                                SimulationConstants.WHEEL_RADIUS, // Wheel radius
                                SimulationConstants.STEER_MOI, // Steer MOI
                                SimulationConstants.WHEEL_COF)) // Wheel COF
                        // Configures the track length and track width (spacing between swerve modules)
                        .withTrackLengthTrackWidth(Units.Inches.of(29), Units.Inches.of(29))
                        // Configures the bumper size (dimensions of the robot bumper)
                        .withBumperSize(Units.Inches.of(30), Units.Inches.of(30));
                
                
                /* Create a swerve drive simulation */ // FIX
                this.swerveDriveSimulation = new SwerveDriveSimulation(
                // Specify Configuration
                driveTrainSimulationConfig,
                // Specify starting pose - use the same starting pose based on alliance
                startingPose);

                // Register with simulation world
                SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);

                gyroIO = new GyroIOSim(this.swerveDriveSimulation.getGyroSimulation());
                moduleIOs = new ModuleIO[4];
                int i;
                for(i=0;i<4;i++) moduleIOs[i] = new ModuleIOSim(swerveDriveSimulation.getModules()[i]);
            }
        } 
        catch(IOException e) {
            throw new RuntimeException(e);
        }
        
        // Enable heading correction and cosine compensator
        this.swerveDrive.setHeadingCorrection(true);
        this.swerveDrive.setCosineCompensator(true);
        
        // Unsure what this really does, so it will remain commented out
        /*this.swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
        this.swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
        */
    }

    @Override
    public void periodic() {
        // Validate gyro before updating - do we want this ? if the gyro is not valid, we should not be updating the odometry?
        //At the very least we want to know if the gyro is bad asap.  We should discuss.
        if (swerveDrive.getGyro() == null) {
            SmartDashboard.putBoolean("Swerve/GyroValid", false);
            return;
        }
        SmartDashboard.putBoolean("Swerve/GyroValid", true);

        //Primary feedback loop for the swerve drive with encoders/gryo
        // Update odometry with latest module states and gyro reading this should be run in every loop
        swerveDrive.updateOdometry(); 

        // Track alliance and pose for debugging
        var alliance = DriverStation.getAlliance();
        boolean isBlue = !alliance.isPresent() || alliance.get() == DriverStation.Alliance.Blue;
        SmartDashboard.putString("Swerve/CurrentAlliance", isBlue ? "BLUE" : "RED");
        SmartDashboard.putBoolean("Swerve/IsSimulation", Robot.isSimulation());
        
        // Secondary feedback loop for the swerve drive with vision
        // 2. If vision target visible & data fresh, use as correction
        if(RobotContainer.visionSubsystem.isTargetVisible() && 
           RobotContainer.visionSubsystem.getImageAge() < VisionConstants.MAX_ACCEPTABLE_DELAY) {
            
            PhotonTrackedTarget target = RobotContainer.visionSubsystem.getTarget();
            if (target != null) {
                Transform3d targetPose = target.getBestCameraToTarget();
                
                // Calculate vision measurement standard deviations
                Matrix<N3, N1> visionStdDevs = calculateVisionStdDevs(target);
                
                // This makes testing much more difficult than it needs to be
                /*swerveDrive.addVisionMeasurement(
                    calculateRobotPoseFromVision(targetPose),
                    Timer.getFPGATimestamp() - (RobotContainer.visionSubsystem.getImageAge() / 1000.0),
                    visionStdDevs
                );*/
                
                // Log std devs for tuning
                SmartDashboard.putNumber("Vision/StdDev", visionStdDevs.get(0, 0));
            }
        }

        Logger.recordOutput("Drive/Pose", swerveDrive.getPose());
        Pose2d currentPose = swerveDrive.getPose();
        Logger.recordOutput("Drive/Pose", currentPose);
        Logger.recordOutput("FieldSimulation/RobotPose", new Pose3d(currentPose));
        Logger.recordOutput("FieldSimulation/RobotPose", new Pose3d(swerveDrive.getPose()));

        // Update the encoder positions
        SwerveModule[] modules = this.swerveDrive.getModules();
        SmartDashboard.putNumber("[SWERVE] Front Left Encoder Position",  modules[0].getRawAbsolutePosition());
        SmartDashboard.putNumber("[SWERVE] Front Right Encoder Position", modules[1].getRawAbsolutePosition());
        SmartDashboard.putNumber("[SWERVE] Back Left Encoder Position",   modules[2].getRawAbsolutePosition());
        SmartDashboard.putNumber("[SWERVE] Back Right Encoder Position",  modules[3].getRawAbsolutePosition());

        // Update the true velocities of all the motors
        SmartDashboard.putNumber("[SWERVE] Front Left Drive Velocity",  modules[0].getDriveMotor().getVelocity());
        SmartDashboard.putNumber("[SWERVE] Front Left Angle Velocity",  modules[0].getAngleMotor().getVelocity());
        SmartDashboard.putNumber("[SWERVE] Front Right Drive Velocity", modules[1].getDriveMotor().getVelocity());
        SmartDashboard.putNumber("[SWERVE] Front Right Angle Velocity", modules[1].getAngleMotor().getVelocity());
        SmartDashboard.putNumber("[SWERVE] Back Left Drive Velocity",   modules[2].getDriveMotor().getVelocity());
        SmartDashboard.putNumber("[SWERVE] Back Left Angle Velocity",   modules[2].getAngleMotor().getVelocity());
        SmartDashboard.putNumber("[SWERVE] Back Right Drive Velocity",  modules[3].getDriveMotor().getVelocity());
        SmartDashboard.putNumber("[SWERVE] Back Right Angle Velocity",  modules[3].getAngleMotor().getVelocity());
    
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

    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }
    
    public void drive(Translation2d translation, double rotation) {
        swerveDrive.drive(translation, rotation, this.fieldRelative, false);
    }

    public void driveBotRelative(Translation2d translation, double rotation) {
        swerveDrive.drive(translation, rotation, false, false);
    }

    public void steer(double steer) {
        swerveDrive.drive(new Translation2d(0, 0), steer * swerveDrive.swerveController.config.maxAngularVelocity,
                          false, false);
    }

    public void autoDrive(ChassisSpeeds speeds) {
        swerveDrive.setChassisSpeeds(speeds);
    }

    public void zeroGyro() {
        this.swerveDrive.zeroGyro();
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d pose) {
        if (pose == null) {
            System.err.println("Warning: Attempted to reset odometry with null pose!");
            return;
        }
        
        try {
            // First zero the gyro
            zeroGyro();
            
            // Small delay to ensure gyro has time to zero
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                // Ignore interrupted exception
            }
            
            // Then reset the odometry with the new pose
            swerveDrive.resetOdometry(pose);
        } catch (Exception e) {
            System.err.println("Error resetting odometry: " + e.getMessage());
            e.printStackTrace();
        }
    }

    public void setFieldRelative(boolean fieldRelative) {
        this.fieldRelative = fieldRelative;
        SmartDashboard.putBoolean("[SWERVE] Field Relative", this.fieldRelative);
    }
    
    public boolean isFieldRelative() {
        return fieldRelative;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return swerveDrive.getRobotVelocity();
    }
    public SwerveDrive getSwerveDrive() {
        return this.swerveDrive;
    }

    public SwerveDriveSimulation getSwerveDriveSimulation() {
        return swerveDriveSimulation;
    }

    private Pose2d calculateRobotPoseFromVision(Transform3d targetToCamera) {
        // Get the camera position relative to robot center
        Transform3d robotToCamera = new Transform3d(
            new Translation3d(
                VisionConstants.CAMERA_POSITION_X,
                VisionConstants.CAMERA_POSITION_Y,
                VisionConstants.CAMERA_POSITION_Z
            ),
            new Rotation3d(
                VisionConstants.CAMERA_PITCH_RADIANS,
                VisionConstants.CAMERA_YAW_RADIANS,
                VisionConstants.CAMERA_ROLL_RADIANS
            )
        );

        // Convert 3D transform to 2D
        Transform2d transform2d = new Transform2d(
            new Translation2d(targetToCamera.getX(), targetToCamera.getY()),
            new Rotation2d(targetToCamera.getRotation().getZ())
        );

        // Create pose from transform
        Pose2d cameraPose = new Pose2d().transformBy(transform2d);
        
        // Account for camera offset
        Transform2d cameraOffset = new Transform2d(
            new Translation2d(robotToCamera.getX(), robotToCamera.getY()),
            new Rotation2d(robotToCamera.getRotation().getZ())
        );
        
        return cameraPose.transformBy(cameraOffset);
    }

    //calculate the standard deviation of the vision data based on the target area and ambiguity
    private Matrix<N3, N1> calculateVisionStdDevs(PhotonTrackedTarget target) {
     
        double area = target.getArea();
        double ambiguity = target.getPoseAmbiguity();
        
        // More area = more accurate, more ambiguity = less accurate
        double baseStdDev = VisionConstants.BASE_VISION_STD_DEV;
        double areaFactor = VisionConstants.AREA_TO_STD_DEV_FACTOR / area;
        double totalStdDev = baseStdDev * (1 + areaFactor) * (1 + ambiguity);
        
        // Clamp to reasonable values
        totalStdDev = Math.min(Math.max(totalStdDev, 
                                       VisionConstants.MIN_VISION_STD_DEV),
                                       VisionConstants.MAX_VISION_STD_DEV);
        
        // Create 3x1 matrix with calculated std dev
        return new Matrix<>(
            Nat.N3(),
            Nat.N1(),
            new double[] {totalStdDev, totalStdDev, totalStdDev * 2} // Higher rotation uncertainty
        );
    }

}