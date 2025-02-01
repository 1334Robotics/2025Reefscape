package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DirectionSnapConstants;
import frc.robot.RobotContainer;

public class DirectionSnapSubsystem extends SubsystemBase {
    private double    targetYaw;
    private boolean   hitTargetYaw;
    private final PID pidController;
    
    public DirectionSnapSubsystem() {
        this.hitTargetYaw = true; // This should disable it until it is requested
        pidController = new PID(
            DirectionSnapConstants.PID_KD,
            DirectionSnapConstants.PID_KI,
            DirectionSnapConstants.PID_KD,
            DirectionSnapConstants.PID_TAU,
            DirectionSnapConstants.PID_LIM_MIN,
            DirectionSnapConstants.PID_LIM_MAX,
            DirectionSnapConstants.PID_LIM_MIN_INT,
            DirectionSnapConstants.PID_LIM_MAX_INT,
            DirectionSnapConstants.PID_SAMPLE_TIME
        );
    }

    public void snap(Direction direction) {
        this.targetYaw = direction.degrees;
        this.hitTargetYaw = false;
    }

    public boolean isAtTarget() {
        return !this.hitTargetYaw;
    }

    @Override
    public void periodic() {
        // Rotate towards the target yaw
        if(!hitTargetYaw) {
            SmartDashboard.putBoolean("[DIRECTIONSNAP] Turning", true);

            // Calculate the error
            double yaw = RobotContainer.gyroSubsystem.getData().yaw;
            double error = targetYaw - yaw;
            SmartDashboard.putNumber("[DIRECTIONSNAP] Error", error);

            // Turn the robot towards the targetYaw
            pidController.update(targetYaw, yaw);
            double steer = pidController.getSteer();
            SmartDashboard.putNumber("[DIRECTIONSNAP] Steer", steer);
            RobotContainer.swerveSubsystem.steer(steer);

            // Check if the yaw is within acceptable range
            if(Math.abs(error) <= DirectionSnapConstants.MAX_ACCEPTABLE_YAW_ERROR)
                this.hitTargetYaw = true;
        } else {
            SmartDashboard.putBoolean("[DIRECTIONSNAP] Turning", false);
            SmartDashboard.putNumber("[DIRECTIONSNAP] Error", 0);
            SmartDashboard.putNumber("[DIRECTIONSNAP] Steer", 0);
        }
    }
}