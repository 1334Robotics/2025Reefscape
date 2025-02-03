package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveConstants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

import java.io.File;
import java.io.IOException;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private boolean fieldRelative;
    private SwerveDriveSimulation swerveDriveSimulation;

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

        // Simulation initialization - only do this in simulation mode
        if (RobotBase.isSimulation()) {
            DriveTrainSimulationConfig basicConfig = DriveTrainSimulationConfig.Default();
            this.swerveDriveSimulation = new SwerveDriveSimulation(
                driveTrainSimulationConfig,
                new Pose2d(3, 3, new Rotation2d())
            );
            
            // Register with simulation world
            SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
        }
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