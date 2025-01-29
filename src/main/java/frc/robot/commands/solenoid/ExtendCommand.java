package frc.robot.commands.solenoid;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;



public class ExtendCommand extends Command {
    public ExtendCommand() {
        addRequirements(RobotContainer.solenoidSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.solenoidSubsystem.extend();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
