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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final AprilTagFieldLayout tagLayout;
    private PhotonPoseEstimator poseEstimator;
    private final Field2d m_field = new Field2d();
    private Pose2d lastPose = new Pose2d();
    private boolean hasLoggedSimWarning = false;
    
    // NetworkTables
    private final NetworkTable visionTable;
    
    // Simulation-specific components
    private VisionSystemSim visionSim;
    private PhotonCameraSim cameraSim;

    public VisionSubsystem() {
        // Initialize NetworkTables
        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        
        // Initialize SmartDashboard entries under Vision table
        visionTable.getEntry("HasTarget").setBoolean(false);
        visionTable.getEntry("TargetID").setDouble(-1);
        visionTable.getEntry("TargetYaw").setDouble(0.0);
        visionTable.getEntry("TargetPitch").setDouble(0.0);
        visionTable.getEntry("TargetAmbiguity").setDouble(-1);
        
        // Add Field2d widget to Vision table
        NetworkTableInstance.getDefault().getTable("Vision").putData("Field", m_field);
        
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
        try {
            if (camera == null) {
                if (!hasLoggedSimWarning) {
                    System.out.println("Camera not initialized in VisionSubsystem");
                    hasLoggedSimWarning = true;
                }
                return;
            }

            var result = camera.getLatestResult();
            boolean hasTarget = result != null && result.hasTargets();
            visionTable.getEntry("HasTarget").setBoolean(hasTarget);
            
            if (hasTarget) {
                PhotonTrackedTarget target = result.getBestTarget();
                if (target != null) {
                    double ambiguity = target.getPoseAmbiguity();
                    
                    if (ambiguity < VisionConstants.MAX_AMBIGUITY) {
                        visionTable.getEntry("TargetID").setDouble(target.getFiducialId());
                        visionTable.getEntry("TargetYaw").setDouble(target.getYaw());
                        visionTable.getEntry("TargetPitch").setDouble(target.getPitch());
                        visionTable.getEntry("TargetAmbiguity").setDouble(ambiguity);
                        
                        updatePoseEstimation();
                    }
                }
            } else {
                // Clear target data when no target is visible
                visionTable.getEntry("TargetID").setDouble(-1);
                visionTable.getEntry("TargetYaw").setDouble(0.0);
                visionTable.getEntry("TargetPitch").setDouble(0.0);
                visionTable.getEntry("TargetAmbiguity").setDouble(-1);
            }
        } catch (Exception e) {
            System.err.println("Error in VisionSubsystem periodic: " + e.toString());
            e.printStackTrace();
        }
    }

    private void updatePoseEstimation() {
        try {
            if (poseEstimator == null) {
                return;
            }
            Pose2d currentPose = getCurrentPose();
            m_field.setRobotPose(currentPose);
            lastPose = currentPose;
        } catch (Exception e) {
            System.err.println("Error updating pose estimation: " + e.toString());
            e.printStackTrace();
        }
    }

    private Pose2d getCurrentPose() {
        try {
            Optional<EstimatedRobotPose> result = getEstimatedGlobalPose(lastPose);
            return result.map(pose -> pose.estimatedPose.toPose2d()).orElse(lastPose);
        } catch (Exception e) {
            System.err.println("Error getting current pose: " + e.toString());
            e.printStackTrace();
            return lastPose;
        }
    }

    @Override
    public void simulationPeriodic() {
        if (RobotBase.isSimulation()) {
            try {
                if (visionSim == null || cameraSim == null) {
                    if (!hasLoggedSimWarning) {
                        System.out.println("Vision simulation not fully initialized");
                        hasLoggedSimWarning = true;
                    }
                    return;
                }

                visionSim.update(lastPose);
                
                var result = camera.getLatestResult();
                if (result != null && result.hasTargets()) {
                    visionTable.getEntry("SimTargetCount").setDouble(result.getTargets().size());
                } else {
                    visionTable.getEntry("SimTargetCount").setDouble(0);
                }
            } catch (Exception e) {
                System.err.println("Error in vision simulation: " + e.toString());
                e.printStackTrace();
            }
        }
    }


    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        try {
            if (poseEstimator == null || camera == null) {
                return Optional.empty();
            }
            poseEstimator.setReferencePose(prevEstimatedRobotPose);
            var result = camera.getLatestResult();
            if (result == null) {
                return Optional.empty();
            }
            return poseEstimator.update(result);
        } catch (Exception e) {
            System.err.println("Error getting estimated global pose: " + e.toString());
            e.printStackTrace();
            return Optional.empty();
        }
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