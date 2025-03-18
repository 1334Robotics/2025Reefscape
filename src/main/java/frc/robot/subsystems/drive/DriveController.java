package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DriveController extends SubsystemBase {
    public enum Controller {
        MANUAL(1),
        DIRECTIONSNAP(2),
        TAGTRACKING(3);

        public final int priority;
        private Controller(int priority) {
            this.priority = priority;
        }
    }

    // This is a bit field where controllers may request and relinquish control.
    // The highest value bit dictates the controller currently allowed to control the drivetrain
    private long controller = 0;

    public DriveController() {
        this.requestControl(Controller.MANUAL);
    }

    public void takeAllControlAway() {
        this.controller = 0;
        this.requestControl(Controller.MANUAL);
    }

    public boolean requestControl(Controller controller) {
        this.controller |= (1L << controller.priority);
        long highestPriorityBit = Long.highestOneBit(this.controller);
        return highestPriorityBit == (1L << controller.priority);
    }

    public void relinquishControl(Controller controller) {
        this.controller &= ~(1L << controller.priority);
        if(this.controller != 0) this.controller = Long.highestOneBit(this.controller);
    }

    private boolean hasControl(Controller controller) {
        return (1L << controller.priority) == Long.highestOneBit(this.controller);
    }

    public void drive(Controller controller, Translation2d translation, double rotation) {
        if(!this.hasControl(controller)) return;

        RobotContainer.swerveSubsystem.drive(translation, rotation);
    }

    public void steer(Controller controller, double steer) {
        if(!this.hasControl(controller)) return;

        RobotContainer.swerveSubsystem.steer(steer);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[SWERVE] Controller", controller);
        SmartDashboard.putNumber("[SWERVE] Controller With Control", Long.highestOneBit(this.controller));
    }
}
