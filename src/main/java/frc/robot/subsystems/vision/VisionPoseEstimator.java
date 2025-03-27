package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VisionPoseEstimator extends SubsystemBase {
    private final PhotonCamera cam;
    private final PhotonPoseEstimator photonPoseEstimator;
    public VisionPoseEstimator() {
        cam = new PhotonCamera("Cam1");
        //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
        //CHANGE TO ACTURAL VALUES VERY IMPORTANT, We also seemed to missed this when we made the first one
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        
        // Get all unread results
        List<PhotonPipelineResult> results = cam.getAllUnreadResults();
        
        // Use the latest result if available
        if (!results.isEmpty()) {
            return photonPoseEstimator.update(results.get(results.size() - 1));
        }
        
        // Return an empty Optional if no results are available
        return Optional.empty();
    }    
}
