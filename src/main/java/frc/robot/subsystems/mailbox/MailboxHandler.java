package frc.robot.subsystems.mailbox;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.MailboxConstants;
import frc.robot.subsystems.elevator.ElevatorLevel;

public class MailboxHandler extends SubsystemBase {
    private boolean allowShoot;
    private boolean feeding;
    public  boolean allowFeeding;
    private final LaserCanSubsystem outputLaserCan;
    private final LaserCanSubsystem inputLaserCan;

    public MailboxHandler() {
        this.allowShoot   = false;
        this.feeding      = false;
        this.allowFeeding = false;
        this.outputLaserCan = new LaserCanSubsystem(MailboxConstants.LASERCAN_OUTPUT_ID, MailboxConstants.LASERCAN_OUTPUT_CLOSE_READING);
        this.outputLaserCan.configureDevice(new Rectangle(MailboxConstants.LASERCAN_OUTPUT_X,
                                                          MailboxConstants.LASERCAN_OUTPUT_Y,
                                                          MailboxConstants.LASERCAN_OUTPUT_HEIGHT,
                                                          MailboxConstants.LASERCAN_OUTPUT_WIDTH));
        this.inputLaserCan = new LaserCanSubsystem(MailboxConstants.LASERCAN_INPUT_ID, MailboxConstants.LASERCAN_INPUT_CLOSE_READING);
        this.inputLaserCan.configureDevice(new Rectangle(MailboxConstants.LASERCAN_INPUT_X,
                                                         MailboxConstants.LASERCAN_INPUT_Y,
                                                         MailboxConstants.LASERCAN_INPUT_HEIGHT,
                                                         MailboxConstants.LASERCAN_INPUT_WIDTH));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("[MAILBOX] Allow Shooting", this.allowFeeding);

        // Check for a coral waiting to be inputted and feed it in
        // Feeding currently doesn't work.
        // It feeds until the output doesn't see it, not until the input doesn't see it
        if(this.allowFeeding) {
            if(!this.outputLaserCan.inRange()) {
                this.feeding = true;
                this.allowShoot = false;
                RobotContainer.mailboxSubsystem.feed();
                return;
            } else if(this.feeding && !this.inputLaserCan.inRange()) {
                RobotContainer.mailboxSubsystem.stop();
                this.allowShoot = false;
                this.feeding = false;
                return;
            }
        } else this.feeding = false;

        this.allowFeeding = false;
        if(this.outputLaserCan.inRange()) {
            if(this.allowShoot) {
                // Ensure that the level is valid (L1-L4)
                ElevatorLevel level = RobotContainer.elevatorHandler.getLevel();
                if(level == null || level == ElevatorLevel.BOTTOM || level == ElevatorLevel.FEED) return;

                // Shoots it on high for L2+; on low for L1
                RobotContainer.mailboxSubsystem.output((level != ElevatorLevel.L1));
                this.allowShoot = false;
            }
        } else {
            RobotContainer.mailboxSubsystem.stop();
            this.allowShoot = false;
        }
    }

    public void allowShoot() {
        this.allowShoot = true;
    }
}
