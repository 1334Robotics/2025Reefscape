package frc.robot.subsystems.intake;

public interface IntakeIO {

    public void setRunning(boolean runIntake);

    public boolean isNoteInsideIntake();

    public void launchNote();
}
