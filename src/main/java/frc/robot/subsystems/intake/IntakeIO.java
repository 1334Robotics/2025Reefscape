package frc.robot.subsystems.simulation.intake;

public interface IntakeIO {

    public void setRunning(boolean runIntake);

    public boolean isNoteInsideIntake();

    public void launchNote();
}
