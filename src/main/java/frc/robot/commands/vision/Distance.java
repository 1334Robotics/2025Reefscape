package frc.robot.commands.vision;

public class Distance {
    /**
     * Imagine this top-down view where R is the robot, # is the tag, and (=, |) are the walls
     * 
     * |==========|
     * |          |
     * |      #   |
     * |          |
     * |          |
     * |        R |
     * |==========|
     * 
     * When looking at the field from above, the "x" distance is the horizontal distance from the tag (2),
     * and the "y" distance is the vertical distance from the tag (3)
     */

    // These values should be in cm
    public final double x;
    public final double y;

    public Distance(double x, double y) {
        this.x = x;
        this.y = y;
    }
}