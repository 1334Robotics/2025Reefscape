package frc.robot.subsystems.drive;

public enum Direction {
    FORWARDS(0),
    LEFT(270),
    RIGHT(90),
    BACKWARDS(180);

    public final int degrees;
    Direction(int degrees) {
        this.degrees = degrees;
    }
}