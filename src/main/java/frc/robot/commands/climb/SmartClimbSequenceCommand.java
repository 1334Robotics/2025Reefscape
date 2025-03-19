package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.constants.ClimbConstants;

public class SmartClimbSequenceCommand extends SequentialCommandGroup {
    public SmartClimbSequenceCommand() {
        addCommands(
            // Unlock climb mechanism and wait until motors reach position
            new UnlockClimbCommand(),
            new WaitUntilCommand(() -> {
                double lockPos = RobotContainer.climbSubsystem.getLockMotorPositionInches();
                return lockPos <= ClimbConstants.UNLOCK_POS; // Adjust threshold as needed
            }),
            
            // Force pins down and wait until they reach position
            new ForcePinsDownCommand(),
            new WaitUntilCommand(() -> {
                double motor1Pos = RobotContainer.climbSubsystem.getLowerMotor1PositionInches();
                double motor2Pos = RobotContainer.climbSubsystem.getLowerMotor2PositionInches();
                return motor1Pos >= ClimbConstants.MOTOR1_POS && motor2Pos >= ClimbConstants.MOTOR2_POS; // Adjust thresholds as needed
            }),
            
            // Lock climb mechanism and wait until secured
            new LockClimbCommand(),
            new WaitUntilCommand(() -> {
                double lockPos = RobotContainer.climbSubsystem.getLockMotorPositionInches();
                return lockPos >= ClimbConstants.LOCK_POS; // Adjust threshold as needed
            }),
            
            // Stop all motors once complete
            new StopClimbCommand()
        );
    }
}
