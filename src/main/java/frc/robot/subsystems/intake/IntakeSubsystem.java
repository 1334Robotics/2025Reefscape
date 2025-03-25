package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    public boolean isCoralInsideIntake() {
        return io.isCoralInsideIntake();
    }

    public void launchCoral() {
        io.launchCoral();
    }

    public boolean isRunning() {
        return io.isRunning();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Intake/Running", isRunning());
        Logger.recordOutput("Intake/HasCoral", isCoralInsideIntake());
    }
}
