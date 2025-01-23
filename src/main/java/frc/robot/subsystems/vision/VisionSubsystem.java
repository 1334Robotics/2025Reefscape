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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final AprilTagFieldLayout tagLayout;
    private final PhotonPoseEstimator poseEstimator;
    private final Field2d m_field = new Field2d();
    
    // NetworkTables structure
    private final NetworkTable visionTable;
    private final NetworkTableEntry hasTargetEntry;
    private final NetworkTableEntry targetIdEntry;
    private final NetworkTableEntry targetYawEntry;
    private final NetworkTableEntry targetPitchEntry;
    private final NetworkTableEntry targetAmbiguityEntry;
    private final NetworkTableEntry poseXEntry;
    private final NetworkTableEntry poseYEntry;
    private final NetworkTableEntry poseRotEntry;
    private final NetworkTableEntry latencyMsEntry;
    
    // Mutable state
    private Pose2d lastPose = new Pose2d();
    private boolean hasLoggedSimWarning = false;
    
    // Simulation-specific components
    private final VisionSystemSim visionSim;
    private final PhotonCameraSim cameraSim;
    private final NetworkTableEntry simTargetCountEntry;

    public VisionSubsystem() {
        // Initialize NetworkTables with hierarchical structure
        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        
        // Target detection entries
        NetworkTable targetTable = visionTable.getSubTable("target");
        hasTargetEntry = targetTable.getEntry("hasTarget");
        targetIdEntry = targetTable.getEntry("id");
        targetYawEntry = targetTable.getEntry("yaw");
        targetPitchEntry = targetTable.getEntry("pitch");
        targetAmbiguityEntry = targetTable.getEntry("ambiguity");
        
        // Pose estimation entries
        NetworkTable poseTable = visionTable.getSubTable("pose");
        poseXEntry = poseTable.getEntry("x");
        poseYEntry = poseTable.getEntry("y");
        poseRotEntry = poseTable.getEntry("rotation");
        
        // Performance metrics
        NetworkTable metricsTable = visionTable.getSubTable("metrics");
        latencyMsEntry = metricsTable.getEntry("latencyMs");
        
        // Simulation entries
        NetworkTable simTable = visionTable.getSubTable("sim");
        simTargetCountEntry = simTable.getEntry("targetCount");
        
        // Initialize all entries with default values
        initializeNetworkTableEntries();
        
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

            // Initialize simulation if needed
            if (RobotBase.isSimulation()) {
                System.out.println("Initializing vision simulation");
                var simComponents = initializeSimulation();
                visionSim = simComponents.visionSim;
                cameraSim = simComponents.cameraSim;
            } else {
                visionSim = null;
                cameraSim = null;
            }
            
            // Add Field2d widget to SmartDashboard
            SmartDashboard.putData("Field", m_field);
        } catch (Exception e) {
            System.err.println("Error initializing vision subsystem: " + e.getMessage());
            e.printStackTrace();
            throw new RuntimeException("Failed to load AprilTag layout: " + e.getMessage());
        }
    }

    private void initializeNetworkTableEntries() {
        // Initialize with default values
        hasTargetEntry.setBoolean(false);
        targetIdEntry.setDouble(-1);
        targetYawEntry.setDouble(0.0);
        targetPitchEntry.setDouble(0.0);
        targetAmbiguityEntry.setDouble(-1);
        
        poseXEntry.setDouble(0.0);
        poseYEntry.setDouble(0.0);
        poseRotEntry.setDouble(0.0);
        
        latencyMsEntry.setDouble(0.0);
        
        if (RobotBase.isSimulation()) {
            simTargetCountEntry.setDouble(0);
        }
    }

    private static class SimComponents {
        final VisionSystemSim visionSim;
        final PhotonCameraSim cameraSim;

        SimComponents(VisionSystemSim visionSim, PhotonCameraSim cameraSim) {
            this.visionSim = visionSim;
            this.cameraSim = cameraSim;
        }
    }

    private SimComponents initializeSimulation() {
        var visionSim = new VisionSystemSim("main");
        
        var cameraProp = new SimCameraProperties();
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

        var cameraSim = new PhotonCameraSim(camera, cameraProp);
        visionSim.addCamera(cameraSim, VisionConstants.ROBOT_TO_CAMERA);
        visionSim.addAprilTags(tagLayout);
        
        System.out.println("Vision simulation initialized successfully");
        return new SimComponents(visionSim, cameraSim);
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
            hasTargetEntry.setBoolean(hasTarget);
            
            // Update latency metrics
            latencyMsEntry.setDouble(getLatencyMillis());
            
            if (hasTarget) {
                PhotonTrackedTarget target = result.getBestTarget();
                if (target != null) {
                    double ambiguity = target.getPoseAmbiguity();
                    
                    if (ambiguity < VisionConstants.MAX_AMBIGUITY) {
                        // Batch update target information
                        NetworkTableInstance.getDefault().startBatch();
                        try {
                            targetIdEntry.setDouble(target.getFiducialId());
                            targetYawEntry.setDouble(target.getYaw());
                            targetPitchEntry.setDouble(target.getPitch());
                            targetAmbiguityEntry.setDouble(ambiguity);
                        } finally {
                            NetworkTableInstance.getDefault().endBatch();
                        }
                        
                        updatePoseEstimation();
                    }
                }
            } else {
                // Batch clear target data
                NetworkTableInstance.getDefault().startBatch();
                try {
                    targetIdEntry.setDouble(-1);
                    targetYawEntry.setDouble(0.0);
                    targetPitchEntry.setDouble(0.0);
                    targetAmbiguityEntry.setDouble(-1);
                } finally {
                    NetworkTableInstance.getDefault().endBatch();
                }
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
            
            // Batch update pose data
            NetworkTableInstance.getDefault().startBatch();
            try {
                poseXEntry.setDouble(currentPose.getX());
                poseYEntry.setDouble(currentPose.getY());
                poseRotEntry.setDouble(currentPose.getRotation().getDegrees());
            } finally {
                NetworkTableInstance.getDefault().endBatch();
            }
            
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
        if (!RobotBase.isSimulation() || visionSim == null || cameraSim == null) {
            return;
        }

        try {
            visionSim.update(lastPose);
            
            var result = camera.getLatestResult();
            simTargetCountEntry.setDouble(
                result != null && result.hasTargets() ? result.getTargets().size() : 0
            );
        } catch (Exception e) {
            System.err.println("Error in vision simulation: " + e.toString());
            e.printStackTrace();
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
        var result = camera.getLatestResult();
        if (result == null) {
            return 0.0;
        }
        var currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        return (currentTime - result.getTimestampSeconds()) * 1000.0;
    }
}