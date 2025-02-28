package frc.robot.subsystems.simulation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.littletonrobotics.junction.Logger;

/**
 * A simplified simulation subsystem that uses WPILib simulation capabilities
 * instead of IronMaple simulation.
 */
public class SimulationSubsystem extends SubsystemBase {

    private final Field2d field = new Field2d(); // Initialize Field2d
    private final SwerveSubsystem swerveSubsystem;

    /**
     * Creates a new SimulationSubsystem.
     * 
     * @param swerveSubsystem The swerve subsystem to simulate
     */
    public SimulationSubsystem(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        System.out.println("Created simplified simulation subsystem");
    }

    /**
     * Sets the initial pose of the robot in simulation.
     * 
     * @param initialPose The initial pose to set
     */
    public void setInitialPose(Pose2d initialPose) {
        if (Robot.isSimulation()) {
            swerveSubsystem.resetOdometry(initialPose); // Reset swerve odometry
            field.setRobotPose(initialPose);
            System.out.println("Set initial pose to " + initialPose);
        }
    }

    @Override
    public void periodic() {
        // Update field with robot pose
        Pose2d robotPose = RobotContainer.swerveSubsystem.getPose();
        field.setRobotPose(robotPose);
        
        // Log robot pose
        Logger.recordOutput("Simulation/RobotPose", new Pose3d(robotPose));
    }

    /**
     * Gets the field visualization object.
     * 
     * @return The field visualization object
     */
    public Field2d getField() {
        return field;
    }
}
