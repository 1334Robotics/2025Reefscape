package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class LockClimbCommand extends Command {
    private final ClimbSubsystem climb;
    private final double speed;

    public LockClimbCommand(double speed) {
        this.climb = RobotContainer.climbSubsystem;
        this.speed = speed;
        addRequirements(climb);
    }

    @Override
    public void execute() {
        climb.lockClimb(speed);
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
