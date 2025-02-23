package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    public void setRunning(boolean runIntake) {
        io.setRunning(runIntake);
    }

    public boolean isCoralInsideIntake() {
        return io.isCoralInsideIntake();
    }

    public void launchCoral() {
        io.launchCoral();
    }

    @Override
    public void periodic() {
        // Add any periodic updates if needed
    }
}
