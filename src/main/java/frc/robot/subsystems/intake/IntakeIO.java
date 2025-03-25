package frc.robot.subsystems.intake;

public interface IntakeIO {

    public void setRunning(boolean runIntake);

    public boolean isCoralInsideIntake();

    public void launchCoral();

    public boolean isRunning();
}
