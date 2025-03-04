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



public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private boolean fieldRelative;
    private SwerveDriveSimulation swerveDriveSimulation;
    private int count = 0;
    private boolean allowDrive;
    
    // Add speed scaling factor for testing - set to a value between 0.0 and 1.0
    // 0.5 means 50% of normal speed, 0.3 means 30% of normal speed
    private final double TESTING_SPEED_FACTOR = 0.3; // Adjust this value as needed for safe testing
    
    /**
     * Enable vision odometry updates while driving.
     */
    private final boolean     visionDriveTest = false;
    /**
     * PhotonVision class to keep an accurate odometry.
     */
    private VisionSubsystem vision;



    public SwerveSubsystem() {
        this.fieldRelative = false;
        this.allowDrive = true;
        final GyroIO gyroIO;
        final GyroSimulation gyroSimulation;
        final ModuleIO[] moduleIOs;
        SmartDashboard.putBoolean("[SWERVE] Field Relative", this.fieldRelative);
        boolean blueAlliance = false;
        // Set the starting pose based on the alliance color - THIS NEEDS TO BE UPDATED FOR COMPETITION (blue alliance is false) maybe get this from a constant file? or a method?
        Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
                                                                          Meter.of(4)),
                                                        Rotation2d.fromDegrees(0))
                                           : new Pose2d(new Translation2d(Meter.of(16),
                                                                          Meter.of(4)),
                                                        Rotation2d.fromDegrees(180));
        
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
                
                
                /* Create a swerve drive simulation */
                this.swerveDriveSimulation = new SwerveDriveSimulation(
                // Specify Configuration
                driveTrainSimulationConfig,
                // Specify starting pose - this is the starting pose of the robot on the field in simulation
                new Pose2d(5, 7, new Rotation2d(-180))
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
        
    
        
        // Enable heading correction and cosine compensator
        this.swerveDrive.setHeadingCorrection(true);
        this.swerveDrive.setCosineCompensator(true);
        this.swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        this.swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        this.swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
        this.swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
        // swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
        if (visionDriveTest)
        {
        setupPhotonVision();
      // Stop the odometry thread if we are using vision that way we can synchronize updates better.
        swerveDrive.stopOdometryThread();
        }
        setupPathPlanner();
        // Comment out the line that uses RobotModeTriggers
        // RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyro));
    }

  /**
   * Setup the photon vision class.
   */
  public void setupPhotonVision()
  {
    vision = new VisionSubsystem();
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
        if(RobotContainer.visionSubsystem.isTargetVisible() && 
           RobotContainer.visionSubsystem.getImageAge() < VisionConstants.MAX_ACCEPTABLE_DELAY) {
            
            PhotonTrackedTarget target = RobotContainer.visionSubsystem.getTarget();
            if (target != null) {
                Transform3d targetPose = target.getBestCameraToTarget();
                
                // Calculate vision measurement standard deviations
                Matrix<N3, N1> visionStdDevs = calculateVisionStdDevs(target);
                
                swerveDrive.addVisionMeasurement(
                    calculateRobotPoseFromVision(targetPose),
                    Timer.getFPGATimestamp() - (RobotContainer.visionSubsystem.getImageAge() / 1000.0),
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
   /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          swerveDrive::getPose, // Robot pose supplier
          swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
          swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              // Apply speed factor to limit velocity during testing
              ChassisSpeeds scaledSpeeds = new ChassisSpeeds(
                  speedsRobotRelative.vxMetersPerSecond * TESTING_SPEED_FACTOR,
                  speedsRobotRelative.vyMetersPerSecond * TESTING_SPEED_FACTOR,
                  speedsRobotRelative.omegaRadiansPerSecond * TESTING_SPEED_FACTOR
              );
              
              swerveDrive.drive(
                  scaledSpeeds,
                  swerveDrive.kinematics.toSwerveModuleStates(scaledSpeeds),
                  moduleFeedForwards.linearForces() // Keep the original feedforward forces
                               );
            } else
            {
              // Apply speed factor to limit velocity during testing
              ChassisSpeeds scaledSpeeds = new ChassisSpeeds(
                  speedsRobotRelative.vxMetersPerSecond * TESTING_SPEED_FACTOR,
                  speedsRobotRelative.vyMetersPerSecond * TESTING_SPEED_FACTOR,
                  speedsRobotRelative.omegaRadiansPerSecond * TESTING_SPEED_FACTOR
              );
              swerveDrive.setChassisSpeeds(scaledSpeeds);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants - should to be tuned for competition
              new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID constants - should to be tuned for competition
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
      );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }


}