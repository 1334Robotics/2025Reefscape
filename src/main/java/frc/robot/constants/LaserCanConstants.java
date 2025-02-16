package frc.robot.constants;
// Configure the LaserCan's Region of Interest (ROI) to focus readings on a specific area
// Parameters for LaserCan.RegionOfInterest are (x, y, width, height) in pixels
// - x: Horizontal coordinate of the ROI's top-left corner (0 to sensor width - 1)
// - y: Vertical coordinate of the ROI's top-left corner (0 to sensor height - 1)
// - width: Horizontal size of the ROI (must be ≤ sensor width - x)
// - height: Vertical size of the ROI (must be ≤ sensor height - y)

// This example creates a 16x16 pixel ROI starting at position (8,8)
// Suitable for 32x32 sensors: Creates centered ROI (8+16=24 < 32)
// Reduces processing area while maintaining central focus
public class LaserCanConstants {
    public static final int can_id = 0;
    public static final int x = 8;
    public static final int y = 8;
    public static final int width = 16;
    public static final int height = 16;
}
