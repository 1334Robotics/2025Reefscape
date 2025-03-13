package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class UnlockClimbCommand extends Command {
    private final ClimbSubsystem climb;

    public UnlockClimbCommand() {
        this.climb = RobotContainer.climbSubsystem;
        addRequirements(climb);
    }

    @Override
    public void execute() {
        climb.unlockClimb();
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopClimb();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
