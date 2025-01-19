package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.constants.VisionConstants;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final AprilTagFieldLayout tagLayout;
    private PhotonPoseEstimator poseEstimator;
    private final Field2d m_field = new Field2d();
    private Pose2d lastPose = new Pose2d();
    
    // Simulation-specific components
    private VisionSystemSim visionSim;
    private PhotonCameraSim cameraSim;

    public VisionSubsystem() {
        SmartDashboard.putData("Field", m_field);
        System.out.println("Initializing VisionSubsystem");
        camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
        System.out.println("Camera created: " + VisionConstants.CAMERA_NAME);

        try {
            // Load field layout
            tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            System.out.println("AprilTag layout loaded successfully");
            
            // Initialize pose estimator
            poseEstimator = new PhotonPoseEstimator(
                tagLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
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
        visionSim = new VisionSystemSim("main");
        
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

        cameraSim = new PhotonCameraSim(camera, cameraProp);
        visionSim.addCamera(cameraSim, VisionConstants.ROBOT_TO_CAMERA);
        visionSim.addAprilTags(tagLayout);
        
        System.out.println("Vision simulation initialized successfully");
    }

    @Override
    public void periodic() {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            double ambiguity = target.getPoseAmbiguity();
            
            if (ambiguity < VisionConstants.MAX_AMBIGUITY) {
                SmartDashboard.putNumber("Vision/TagID", target.getFiducialId());
                SmartDashboard.putNumber("Vision/Ambiguity", ambiguity);
                SmartDashboard.putNumber("Vision/Yaw", target.getYaw());
                SmartDashboard.putNumber("Vision/Pitch", target.getPitch());
                
                Pose2d currentPose = getCurrentPose();
                m_field.setRobotPose(currentPose);
                lastPose = currentPose;
            }
        }
    }

    private Pose2d getCurrentPose() {
        Optional<EstimatedRobotPose> result = getEstimatedGlobalPose(lastPose);
        return result.map(pose -> pose.estimatedPose.toPose2d()).orElse(lastPose);
    }

    @Override
    public void simulationPeriodic() {
        if (visionSim != null && cameraSim != null) {
            try {
                visionSim.update(lastPose);
                var result = camera.getLatestResult();
                if (result.hasTargets()) {
                    SmartDashboard.putNumber("Vision/SimTargetCount", result.getTargets().size());
                }
            } catch (Exception e) {
                System.err.println("Error in vision simulation: " + e.getMessage());
            }
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (poseEstimator == null) {
            return Optional.empty();
        }
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return poseEstimator.update(camera.getLatestResult());
    }

    public void updateCameraPosition(Transform3d newTransform) {
        if (poseEstimator != null) {
            poseEstimator.setRobotToCameraTransform(newTransform);
        }
        if (RobotBase.isSimulation() && visionSim != null) {
            visionSim.adjustCamera(cameraSim, newTransform);
        }
    }

    public boolean isConnected() {
        return camera.isConnected();
    }

    public double getLatencyMillis() {
        var currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        var timestampSeconds = camera.getLatestResult().getTimestampSeconds();
        return (currentTime - timestampSeconds) * 1000.0; // Convert to milliseconds
    }
}