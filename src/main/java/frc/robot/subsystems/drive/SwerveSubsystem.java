package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.kinematics.ChassisSpeeds;//NEW CAL
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.GyroConstants;
import frc.robot.constants.SimulationConstants;
import frc.robot.constants.SwerveConstants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.imu.SwerveIMU;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIOPigeon2;
import frc.robot.subsystems.gyro.GyroIOSim;
import edu.wpi.first.units.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;
import java.io.File;
import java.io.IOException;
import org.ironmaple.simulation.SimulatedArena;


public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private final VisionSubsystem visionSubsystem;
    private boolean fieldRelative;
    private SwerveDriveSimulation swerveDriveSimulation;
    private int count = 0;
    private boolean allowDrive;
    private final SwerveDriveOdometry odometry;

    private final GyroIO gyroIO;

    public SwerveSubsystem() {
        this.fieldRelative = false;
        this.allowDrive = true;
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

                this.gyroIO = new GyroIOSim(this.swerveDriveSimulation.getGyroSimulation());
                moduleIOs = new ModuleIO[4];
                for (int i = 0; i < 4; i++) {
                    moduleIOs[i] = new ModuleIOSim(swerveDriveSimulation.getModules()[i]);
                }
            } else {
            // Real robot initialization
            this.gyroIO = new GyroIOPigeon2(GyroConstants.DEVICEID); // Assuming GyroConstants.DEVICEID is defined
            moduleIOs = new ModuleIO[4];
            // Initialize moduleIOs for real robot if needed
            }

            // Initialize kinematics and odometry
            SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(29 / 2, 29 / 2), // Front left
                new Translation2d(29 / 2, -29 / 2), // Front right
                new Translation2d(-29 / 2, 29 / 2), // Back left
                new Translation2d(-29 / 2, -29 / 2)  // Back right
            );

            // Initialize module positions (all zeros for starting position)
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
                new SwerveModulePosition(0, new Rotation2d(0)), // Front left
                new SwerveModulePosition(0, new Rotation2d(0)), // Front right
                new SwerveModulePosition(0, new Rotation2d(0)), // Back left
                new SwerveModulePosition(0, new Rotation2d(0))  // Back right
            };

            // Initialize odometry
            this.odometry = new SwerveDriveOdometry(kinematics, gyroIO.getGyroRotation(), modulePositions);

        } catch(IOException e) {
            throw new RuntimeException(e);
        }
        
        // Turn off heading correction and cosine compensation
        this.swerveDrive.setHeadingCorrection(true);
        this.swerveDrive.setCosineCompensator(false);
        }

    @Override
    public void periodic() {
        if (Robot.isSimulation()) {
        // Define wheel circumference
        double wheelCircumference = 2 * Math.PI * 2; // Wheel circumference in meters

        // Get simulated wheel positions
        SwerveModulePosition[] simulatedPositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            double drivePositionMeters = swerveDriveSimulation.getModules()[i].getDriveWheelFinalPosition().in(Units.Rotations) * wheelCircumference;
            Rotation2d steerAngle = swerveDriveSimulation.getModules()[i].getSteerAbsoluteFacing();
            simulatedPositions[i] = new SwerveModulePosition(drivePositionMeters, steerAngle);
        }

        // Update odometry with simulated positions and gyro angle
        Rotation2d gyroAngle = gyroIO.getGyroRotation();
        odometry.update(gyroAngle, simulatedPositions);

        // Update simulation with odometry pose
        Pose2d odometryPose = odometry.getPoseMeters();
        swerveDriveSimulation.setSimulationWorldPose(odometryPose);
    
        // Log simulated pose and odometry pose for debugging
        Logger.recordOutput("Swerve/SimulatedPose", swerveDriveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("Swerve/OdometryPose", odometryPose);
        }

        // Log robot pose and other data
        Logger.recordOutput("Drive/Pose", swerveDrive.getPose());
        Logger.recordOutput("FieldSimulation/RobotPose", new Pose3d(swerveDrive.getPose()));

        // Log wheel positions for debugging
        SwerveModulePosition[] modulePositions = swerveDrive.getModulePositions();
        for (int i = 0; i < modulePositions.length; i++) {
            Logger.recordOutput("Swerve/Module" + i + "/Position", modulePositions[i].distanceMeters);
            Logger.recordOutput("Swerve/Module" + i + "/Angle", modulePositions[i].angle.getDegrees());
        }

        Logger.recordOutput("Swerve/ChassisSpeeds", getChassisSpeeds());

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

    public void autoDrive(ChassisSpeeds speeds) {
        swerveDrive.drive(speeds);
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

    public void autoDrive(ChassisSpeeds speeds) {
        swerveDrive.drive(speeds);
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
}