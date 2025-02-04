package frc.robot.subsystems.drive;

public enum Direction {
    FORWARDS(0),
    LEFT(90),
    RIGHT(270),
    BACKWARDS(180);

    public final int degrees;
    Direction(int degrees) {
        this.degrees = degrees;
    }
}