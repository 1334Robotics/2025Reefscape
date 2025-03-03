package frc.robot.subsystems.mailbox;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.MailboxConstants;

public class MailboxHandler extends SubsystemBase {
    private boolean allowShoot;
    private boolean feeding;
    public  boolean forceFeeding;
    private final LaserCanSubsystem outputLaserCan;
    private final LaserCanSubsystem inputLaserCan;

    public MailboxHandler() {
        this.allowShoot   = false;
        this.feeding      = false;
        this.forceFeeding = false;
        this.outputLaserCan = new LaserCanSubsystem(MailboxConstants.LASERCAN_OUTPUT_ID, MailboxConstants.LASERCAN_CLOSE_READING);
        this.inputLaserCan  = new LaserCanSubsystem(MailboxConstants.LASERCAN_INPUT_ID, MailboxConstants.LASERCAN_CLOSE_READING);
    }

    @Override
    public void periodic() {
        // Check for a coral waiting to be inputted and feed it in
        if(this.inputLaserCan.inRange() || this.forceFeeding) {
            if(!this.outputLaserCan.inRange()) {
                this.feeding = true;
                this.allowShoot = false;
                RobotContainer.mailboxSubsystem.feed();
                return;
            } else if(this.feeding) {
                RobotContainer.mailboxSubsystem.stop();
                this.allowShoot = false;
                this.feeding = false;
                return;
            }
        } else this.feeding = false;

        // Later, get the elevator level and decide what speed to run the output at
        // Low is for L1, high is for all others
        this.forceFeeding = false;
        if(this.outputLaserCan.inRange()) {
            if(this.allowShoot) {
                RobotContainer.mailboxSubsystem.output(true);
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
