package frc.robot.auto;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.AutoConstants;

public enum AutoStartPosition {
    RED_1(AutoConstants.RED_LEFT_STARTING_POSE), // Unsure if left is correct here; verify
    RED_2(AutoConstants.RED_CENTER_STARTING_POSE),
    RED_3(AutoConstants.RED_RIGHT_STARTING_POSE),
    BLUE_1(AutoConstants.BLUE_LEFT_STARTING_POSE),
    BLUE_2(AutoConstants.BLUE_CENTER_STARTING_POSE),
    BLUE_3(AutoConstants.BLUE_RIGHT_STARTING_POSE);

    private final Pose2d startPose2d;
    
    private AutoStartPosition(Pose2d startPose2d) {
        this.startPose2d = startPose2d;
    }

    public Pose2d getStartPose() {
        return this.startPose2d;
    }

    @Override
    public String toString() {
        return switch(this) {
            case RED_1  -> "Red 1";
            case RED_2  -> "Red 2";
            case RED_3  -> "Red 3";
            case BLUE_1 -> "Blue 1";
            case BLUE_2 -> "Blue 2";
            case BLUE_3 -> "Blue 3";
        };
    }

    public static AutoStartPosition getPositionFromDriverStation() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        int location = DriverStation.getLocation().orElse(0);
        if(!alliance.isPresent() || location == 0) return null;

        if(alliance.get() == DriverStation.Alliance.Blue) {
            return switch(location) {
                case 1  -> BLUE_1;
                case 2  -> BLUE_2;
                case 3  -> BLUE_3;
                default -> null;
            };
        } else {
            return switch(location) {
                case 1  -> RED_1;
                case 2  -> RED_2;
                case 3  -> RED_3;
                default -> null;
            };
        }
    }
}
