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
import frc.robot.Robot;
import frc.robot.constants.SimulationConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.imu.SwerveIMU; //NEW CAL
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
import org.ironmaple.simulation.SimulatedArena;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private final VisionSubsystem visionSubsystem;
    private static final double VISION_TRUST_FACTOR = 0.8;//NEW CAL leaving for now was part of a simpler vision implementation
    private boolean fieldRelative;
    private SwerveDriveSimulation swerveDriveSimulation;
    private int count = 0;
    private boolean allowDrive;

    public SwerveSubsystem(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
        this.fieldRelative = false;
        this.allowDrive = true;
        final GyroIO gyroIO;
        final GyroSimulation gyroSimulation;
        final ModuleIO[] moduleIOs;
        SmartDashboard.putBoolean("[SWERVE] Field Relative", this.fieldRelative);
 
        
        // Create the swerve drive
        File swerveDirectory = new File(Filesystem.getDeployDirectory(), SwerveConstants.SWERVE_DRIVE_DIRECTORY);
        try {
            this.swerveDrive = new SwerveParser(swerveDirectory).createSwerveDrive(SwerveConstants.MAX_SPEED);

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
                
                
                /* Create a swerve drive simulation */
                this.swerveDriveSimulation = new SwerveDriveSimulation(
                // Specify Configuration
                driveTrainSimulationConfig,
                // Specify starting pose
                new Pose2d(0, 0, new Rotation2d())
                );

                // Register with simulation world
                SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);

                gyroIO = new GyroIOSim(this.swerveDriveSimulation.getGyroSimulation());
                moduleIOs = new ModuleIO[4];
                for (int i = 0; i < 4; i++) {
                    moduleIOs[i] = new ModuleIOSim(swerveDriveSimulation.getModules()[i]);
                }
            }
        } 
        catch(IOException e) {
            throw new RuntimeException(e);
        }
        
        this.swerveDrive.setHeadingCorrection(true);
        this.swerveDrive.setCosineCompensator(true);
    
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

        // Secondary feedback loop for the swerve drive with vision
        // 2. If vision target visible & data fresh, use as correction
        if (visionSubsystem.isTargetVisible() && 
            visionSubsystem.getImageAge() < VisionConstants.MAX_ACCEPTABLE_DELAY) {
            
            PhotonTrackedTarget target = visionSubsystem.getTarget();
            if (target != null) {
                Transform3d targetPose = target.getBestCameraToTarget();
                
                // Calculate vision measurement standard deviations
                Matrix<N3, N1> visionStdDevs = calculateVisionStdDevs(target);
                
                swerveDrive.addVisionMeasurement(
                    calculateRobotPoseFromVision(targetPose),
                    Timer.getFPGATimestamp() - (visionSubsystem.getImageAge() / 1000.0),
                    visionStdDevs
                );
                
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
        if(!this.allowDrive) return;
        swerveDrive.drive(translation, rotation, this.fieldRelative, false);
    }

    public void steer(double steer) {
        swerveDrive.drive(new Translation2d(0, 0), steer * swerveDrive.swerveController.config.maxAngularVelocity,
                          false, false);
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

    //adding odometry reset with gyro zeroing
    public void resetPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
        zeroGyro();  // Optional: zero gyro when resetting pose
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

    public void lockDrive() {
        /*
         * What needs to exist is a ControlManager class which tells either DriveCommand
         * or some autonomous method that it is currently allowed to use the SwerveSubsystem
         */
        this.allowDrive = false;
    }

    public void unlockDrive() {
        this.allowDrive = true;
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