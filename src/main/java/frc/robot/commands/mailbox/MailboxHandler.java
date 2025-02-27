package frc.robot.commands.mailbox;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.MailboxConstants;

public class MailboxHandler extends SubsystemBase {
    public MailboxHandler() {}

    @Override
    public void periodic() {
        // Later, get the elevator level and decide what speed to run the output at
        // Low is for L1, high is for all others
        if(RobotContainer.laserCanSubsystem.isValidMeasurement()
        && RobotContainer.laserCanSubsystem.getDistance() < MailboxConstants.LASERCAN_CLOSE_READING) {
            RobotContainer.mailboxSubsystem.output(true);
        } else {
            RobotContainer.mailboxSubsystem.stop();
        }
    }
}
