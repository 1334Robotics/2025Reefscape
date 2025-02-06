package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.SwerveConstants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIOSim;
import edu.wpi.first.units.Units;
import edu.wpi.first.math.geometry.Rotation2d;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import java.io.File;
import java.io.IOException;

import org.ironmaple.simulation.SimulatedArena;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private boolean fieldRelative;
    private SwerveDriveSimulation swerveDriveSimulation;

    public SwerveSubsystem() {
        this.fieldRelative = false;
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
                                DCMotor.getKrakenX60(4), // Drive motor is a Kraken X60
                                DCMotor.getKrakenX60(1), // Steer motor is a Falcon 500
                                6.12, // Drive motor gear ratio.
                                12.8, // Steer motor gear ratio.
                                Units.Volts.of(0.1), // Drive friction voltage.
                                Units.Volts.of(0.1), // Steer friction voltage
                                Units.Inches.of(2), // Wheel radius
                                Units.KilogramSquareMeters.of(0.03), // Steer MOI
                                1.2)) // Wheel COF
                        // Configures the track length and track width (spacing between swerve modules)
                        .withTrackLengthTrackWidth(Units.Inches.of(24), Units.Inches.of(24))
                        // Configures the bumper size (dimensions of the robot bumper)
                        .withBumperSize(Units.Inches.of(30), Units.Inches.of(30));
                
                
                /* Create a swerve drive simulation */
                this.swerveDriveSimulation = new SwerveDriveSimulation(
                // Specify Configuration
                driveTrainSimulationConfig,
                // Specify starting pose
                new Pose2d(3, 3, new Rotation2d())
                );

                // Register with simulation world
                SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);

                gyroIO = new GyroIOSim(this.swerveDriveSimulation.getGyroSimulation());
                new GyroIOSim(this.swerveDriveSimulation.getGyroSimulation());
                moduleIOs = new ModuleIO[4];
                for (int i = 0; i < 4; i++) {
                    moduleIOs[i] = new ModuleIOSim(swerveDriveSimulation.getModules()[i]);
                }
            }
        } catch(IOException e) {
            throw new RuntimeException(e);
        }
        
        // Turn off heading correction and cosine compensation
        this.swerveDrive.setHeadingCorrection(false);
        this.swerveDrive.setCosineCompensator(false);
        }

    @Override
    public void periodic() {

        // Update the encoder positions
        SwerveModule[] modules = this.swerveDrive.getModules();
        SmartDashboard.putNumber("[SWERVE] Front Left Encoder Position",  modules[0].getAbsolutePosition());
        SmartDashboard.putNumber("[SWERVE] Front Right Encoder Position", modules[1].getAbsolutePosition());
        SmartDashboard.putNumber("[SWERVE] Back Left Encoder Position",   modules[2].getAbsolutePosition());
        SmartDashboard.putNumber("[SWERVE] Back Right Encoder Position",  modules[3].getAbsolutePosition());

        // Update the true velocities of all the motors
        SmartDashboard.putNumber("[SWERVE] Front Left Drive Velocity",  modules[0].getDriveMotor().getVelocity());
        SmartDashboard.putNumber("[SWERVE] Front Left Angle Velocity",  modules[0].getAngleMotor().getVelocity());
        SmartDashboard.putNumber("[SWERVE] Front Right Drive Velocity", modules[1].getDriveMotor().getVelocity());
        SmartDashboard.putNumber("[SWERVE] Front Right Angle Velocity", modules[1].getAngleMotor().getVelocity());
        SmartDashboard.putNumber("[SWERVE] Back Left Drive Velocity",   modules[2].getDriveMotor().getVelocity());
        SmartDashboard.putNumber("[SWERVE] Back Left Angle Velocity",   modules[2].getAngleMotor().getVelocity());
        SmartDashboard.putNumber("[SWERVE] Back Right Drive Velocity",  modules[3].getDriveMotor().getVelocity());
        SmartDashboard.putNumber("[SWERVE] Back Right Angle Velocity",  modules[3].getAngleMotor().getVelocity());
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

    public SwerveDrive getSwerveDrive() {
        return this.swerveDrive;
    }
}