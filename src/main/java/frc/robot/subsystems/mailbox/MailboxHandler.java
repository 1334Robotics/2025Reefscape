package frc.robot.subsystems.mailbox;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.MailboxConstants;

public class MailboxHandler extends SubsystemBase {
    private boolean allowShoot;
    private boolean feeding;
    private final LaserCanSubsystem outputLaserCan;
    private final LaserCanSubsystem inputLaserCan;

    public MailboxHandler() {
        this.allowShoot = false;
        this.feeding    = false;
        this.outputLaserCan = new LaserCanSubsystem(0, MailboxConstants.LASERCAN_CLOSE_READING);
        this.inputLaserCan  = new LaserCanSubsystem(1, MailboxConstants.LASERCAN_CLOSE_READING);
    }

    @Override
    public void periodic() {
        // Check for a coral waiting to be inputted and feed it in
        if(this.inputLaserCan.inRange()) {
            if(!this.outputLaserCan.inRange()) {
                this.feeding = true;
                RobotContainer.mailboxSubsystem.output(true);
            } else if(this.feeding) {
                RobotContainer.mailboxSubsystem.stop();
                this.feeding = false;
                return;
            }
        }

        // Later, get the elevator level and decide what speed to run the output at
        // Low is for L1, high is for all others
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
