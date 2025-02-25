package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class StopClimbCommand extends InstantCommand {
    public StopClimbCommand() {
        super(() -> RobotContainer.climbSubsystem.stopClimb(), RobotContainer.climbSubsystem);
    }
}
