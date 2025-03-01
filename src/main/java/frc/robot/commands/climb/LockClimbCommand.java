package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class LockClimbCommand extends Command {
    private final ClimbSubsystem climb;

    public LockClimbCommand() {
        this.climb = RobotContainer.climbSubsystem;
        addRequirements(climb);
    }

    @Override
    public void execute() {
        climb.lockClimb();
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
