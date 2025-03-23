package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.led.LedHandler.Controller;
import frc.robot.commands.led.LEDColourCommand;

public class TagTrackingHandler extends SubsystemBase {
    private TagTrackingTarget target;

    public TagTrackingHandler() {
        this.target = null;
    }

    public void setTarget(TagTrackingTarget target) {
        this.target = target;
        if(this.target == null) return;
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
        if(this.target == null) return "No Target";
        return switch(this.target) {
            case SCORING_LEFT  -> "Left scoring target";
            case SCORING_RIGHT -> "Right scoring target";
            default            -> "Unknown target";
        };
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("[TAG TRACKING] Target", this.getTargetName());
        
        if (this.target == null) {
            RobotContainer.trackCommand.disable();
            RobotContainer.ledHandler.setColour(Controller.VISION, LEDColourCommand.Colour.RED);
            return;
        }

        RobotContainer.trackCommand.enable();
        
        // Blue when seeing tag, green when aligned
        if (RobotContainer.visionSubsystem.isTargetVisible()) {
            if (RobotContainer.trackCommand.isAligned()) {
                RobotContainer.ledHandler.setColour(Controller.VISION, LEDColourCommand.Colour.GREEN);
            } else {
                RobotContainer.ledHandler.setColour(Controller.VISION, LEDColourCommand.Colour.BLUE); 
            }
        } else {
            RobotContainer.ledHandler.setColour(Controller.VISION, LEDColourCommand.Colour.RED);
        }
    }
}
