package frc.robot.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public class SimulationConstants {
    public static final Voltage DRIVE_FRICTION_VOLTAGE = Units.Volts.of(0.1);
    public static final Voltage STEER_FRICTION_VOLTAGE = Units.Volts.of(0.1);
    public static final Distance WHEEL_RADIUS = Units.Inches.of(2);
    public static final MomentOfInertia STEER_MOI = Units.KilogramSquareMeters.of(0.03);
    public static final double WHEEL_COF = 1.19;
    public static final Distance TRACK_WIDTH = Units.Inches.of(29);
    public static final Distance TRACK_LENGTH = Units.Inches.of(29);
    public static final Distance BYMPER_WIDTH = Units.Inches.of(30);
    public static final Distance BUMPER_LENGTH = Units.Inches.of(30);
    public static final Distance INTAKE_LENGTH = Units.Meters.of(0.4);
    public static final Distance INTAKE_EXTENSION = Units.Meters.of(0.2);
    public static final Distance L3_BRANCH_HEIGHT = Units.Meters.of(1.28);
    public static final Distance L4_BRANCH_HEIGHT = Units.Meters.of(2.1);
    public static final double SCORING_MECH_L3_HEIGHT = 0.35;
    public static final double SCORING_MECH_L4_HEIGHT = 0.46;
    public static final double ROBOT_STARTING_POSE_X = 1.25;
    public static final double ROBOT_STARTING_POSE_Y = 2.25;

    // Elevator simulation constants
    public static final double ELEVATOR_MASS = 5.0; // kg
    public static final double ELEVATOR_GEARING = 10.0;
    public static final double ELEVATOR_DRUM_RADIUS = 0.0254; // meters (1 inch)
    public static final double ELEVATOR_MIN_HEIGHT = 0.0;
    public static final double ELEVATOR_MAX_HEIGHT = 2.0; // meters
    public static final double ELEVATOR_CARRIAGE_MASS = 4.0; // kg
    public static final double ELEVATOR_GRAVITY = 9.81; // m/s^2
    public static final double ELEVATOR_LENGTH = 1.0; // meters
    public static final double ELEVATOR_MOI = 0.1; // kg * m^2
    public static final double ELEVATOR_EFFICIENCY = 0.7;
}
