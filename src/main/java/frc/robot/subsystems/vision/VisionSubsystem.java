package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.EstimatedRobotPose;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import frc.robot.constants.VisionConstants;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final AprilTagFieldLayout tagLayout;
    private PhotonPoseEstimator poseEstimator;
    private final Field2d m_field = new Field2d();
    // Simulation-specific components
    private VisionSystemSim visionSim;
    private PhotonCameraSim cameraSim;

    public VisionSubsystem() {
        SmartDashboard.putData("Field", m_field);
        System.out.println("Initializing VisionSubsystem");
        camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
        System.out.println("Camera created: " + VisionConstants.CAMERA_NAME);

        try {
            // Load 2024 field layout
            tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            System.out.println("AprilTag layout loaded successfully");
            
            // Initialize pose estimator
            poseEstimator = new PhotonPoseEstimator(
                tagLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                camera, 
                VisionConstants.ROBOT_TO_CAMERA
            );

            if (RobotBase.isSimulation()) {
                System.out.println("Initializing vision simulation");
                initializeSimulation();
            }
        } catch (Exception e) {
            System.err.println("Error initializing vision subsystem: " + e.getMessage());
            e.printStackTrace();
            throw new RuntimeException("Failed to load AprilTag layout: " + e.getMessage());
        }
    }

    private void initializeSimulation() {
        // Create vision system simulation
        visionSim = new VisionSystemSim("main");
        
        // Configure camera properties for simulation
        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(
            VisionConstants.CAMERA_RESOLUTION_WIDTH,
            VisionConstants.CAMERA_RESOLUTION_HEIGHT,
            Rotation2d.fromDegrees(VisionConstants.CAMERA_FOV_DEGREES)
        );
        cameraProp.setCalibError(
            VisionConstants.SIM_ERROR_MEAN,
            VisionConstants.SIM_ERROR_STD_DEV
        );
        cameraProp.setFPS(VisionConstants.SIM_CAMERA_FPS);
        cameraProp.setAvgLatencyMs(VisionConstants.SIM_AVERAGE_LATENCY_MS);
        cameraProp.setLatencyStdDevMs(VisionConstants.SIM_LATENCY_STD_DEV_MS);

        // Create camera simulation
        cameraSim = new PhotonCameraSim(camera, cameraProp);
        
        // Add camera and AprilTags to simulation
        visionSim.addCamera(cameraSim, VisionConstants.ROBOT_TO_CAMERA);
        visionSim.addAprilTags(tagLayout);
        
        System.out.println("Vision simulation initialized successfully");
    }

    @Override
    public void periodic() {
        // Get the latest vision data
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            var target = result.getBestTarget();
            System.out.println("Detected AprilTag ID: " + target.getFiducialId() +
                             ", Yaw: " + target.getYaw() +
                             ", Pitch: " + target.getPitch());
            // Update field visualization with latest target data
            m_field.setRobotPose(getCurrentPose());
            System.out.println("Target found: ID=" + target.getFiducialId());
        }
    }

    private Pose2d getCurrentPose() {
        // This is a placeholder - replace with your actual robot pose
        return new Pose2d();

    @Override
    public void simulationPeriodic() {
        if (visionSim != null) {
            // For simulation testing, we'll use a static pose
            // In a real robot, this would come from odometry
            Pose2d robotPose = new Pose2d(3, 3, new Rotation2d());
            visionSim.update(robotPose);
            System.out.println("Vision simulation updated with pose: " + robotPose.toString());
        }
    }

    /**
     * Gets the estimated robot pose if available
     * @param prevEstimatedRobotPose Previous estimated robot pose for reference
     * @return Optional containing the estimated pose, or empty if no estimate available
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (poseEstimator == null) {
            return Optional.empty();
        }
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return poseEstimator.update();
    }

    /**
     * Updates the camera position if it's mounted on a moving mechanism (like a turret)
     * @param newTransform The new robot-to-camera transform
     */
    public void updateCameraPosition(Transform3d newTransform) {
        if (RobotBase.isSimulation() && visionSim != null) {
            visionSim.adjustCamera(cameraSim, newTransform);
        }
    }
}