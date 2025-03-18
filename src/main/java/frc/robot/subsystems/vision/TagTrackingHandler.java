package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.VisionConstants;

public class TagTrackingHandler extends SubsystemBase {
    private TagTrackingTarget target;

    public TagTrackingHandler() {
        this.target = null;
        RobotContainer.trackCommand.schedule();
    }

    public void setTarget(TagTrackingTarget target) {
        this.target = target;
        switch(this.target) {
            case SCORING_LEFT:
                RobotContainer.trackCommand.setRelativeDistance(VisionConstants.LEFT_SCORE_DISTANCE);
                break;
            case SCORING_RIGHT:
                RobotContainer.trackCommand.setRelativeDistance(VisionConstants.RIGHT_SCORE_DISTANCE);
                break;
        }
    }

    private String getTargetName() {
        return switch(this.target) {
            case SCORING_LEFT  -> "Left scoring target";
            case SCORING_RIGHT -> "Right scoring target";
            default            -> "No target";
        };
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("[TAG TRACKING] Target", this.getTargetName());
        if(this.target == null) {
            RobotContainer.trackCommand.disable();
            return;
        } else RobotContainer.trackCommand.enable();
    }
}
