package frc.robot.subsystems.mailbox;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.MailboxConstants;
import frc.robot.subsystems.elevator.LaserCanSubsystem;

public class MailboxHandler extends SubsystemBase {
    private boolean allowShoot;
    //private final LaserCanSubsystem outputLaserCan;

    public MailboxHandler() {
        this.allowShoot = false;
        //this.outputLaserCan = new LaserCanSubsystem(0);
    }

    @Override
    public void periodic() {
        // Later, get the elevator level and decide what speed to run the output at
        // Low is for L1, high is for all others
        /*if(this.outputLaserCan.isValidMeasurement()
        && this.outputLaserCan.getDistance() < MailboxConstants.LASERCAN_CLOSE_READING) {
            if(this.allowShoot) {
                RobotContainer.mailboxSubsystem.output(false);
                this.allowShoot = false;
            }
        } else {
            RobotContainer.mailboxSubsystem.stop();
            this.allowShoot = false;
        }*/
    }

    public void allowShoot() {
        this.allowShoot = true;
    }
}
