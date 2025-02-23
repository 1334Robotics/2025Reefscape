package frc.robot.constants;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.vision.PhotonCameraProperties;
import static edu.wpi.first.units.Units.*;

public class SimulationConstants {
    public static final Voltage DRIVE_FRICTION_VOLTAGE = Units.Volts.of(0.05);
    public static final Voltage STEER_FRICTION_VOLTAGE = Units.Volts.of(0.05);
    public static final Distance WHEEL_RADIUS = Units.Inches.of(2);
    public static final MomentOfInertia STEER_MOI = Units.KilogramSquareMeters.of(0.03);
    public static final double WHEEL_COF = 1.19;
    public static final Distance TRACK_WIDTH = Units.Inches.of(29);
    public static final Distance TRACK_LENGTH = Units.Inches.of(29);
    public static final Distance BUMPER_WIDTH = Units.Inches.of(30);
    public static final Distance BUMPER_LENGTH = Units.Inches.of(30);
    public static final Distance INTAKE_LENGTH = Units.Meters.of(0.4);
    public static final Distance INTAKE_EXTENSION = Units.Meters.of(0.2);
    public static final Distance L3_BRANCH_HEIGHT = Units.Meters.of(1.28);
    public static final Distance L4_BRANCH_HEIGHT = Units.Meters.of(2.1);
    public static final double SCORING_MECH_L3_HEIGHT = 0.35;
    public static final double SCORING_MECH_L4_HEIGHT = 0.46;
    public static final double ROBOT_STARTING_POSE_X = 1.25;
    public static final double ROBOT_STARTING_POSE_Y = 2.25;

    public static final List<PhotonCameraProperties> photonVisionCameras = List.of(
            //            new PhotonCameraProperties(
            //                    "FrontCam",
            //                    Hertz.of(30),
            //                    Milliseconds.of(60),
            //                    Milliseconds.of(5),
            //                    Degrees.of(75),
            //                    0.6,
            //                    0.2,
            //                    1280,
            //                    720,
            //                    new Translation2d(0.3, 0),
            //                    Meters.of(0.7),
            //                    Rotation2d.fromDegrees(0),
            //                    Degrees.of(40),
            //                    Degrees.zero()),
            new PhotonCameraProperties(
                    "FrontLeftCam",
                    Hertz.of(15),
                    Milliseconds.of(80),
                    Milliseconds.of(10),
                    Degrees.of(58),
                    0.9,
                    0.2,
                    1280,
                    800,
                    new Translation2d(0.04, 0.2),
                    Meters.of(0.44),
                    Rotation2d.fromDegrees(0),
                    Degrees.of(-18),
                    Degrees.zero()),
            new PhotonCameraProperties(
                    "FrontRightCam",
                    Hertz.of(15),
                    Milliseconds.of(80),
                    Milliseconds.of(10),
                    Degrees.of(58),
                    0.9,
                    0.2,
                    1280,
                    800,
                    new Translation2d(0.04, -0.2),
                    Meters.of(0.44),
                    Rotation2d.fromDegrees(0),
                    Degrees.of(-18),
                    Degrees.zero())
            //            new PhotonCameraProperties(
            //                    "BackCam",
            //                    Hertz.of(30),
            //                    Milliseconds.of(60),
            //                    Milliseconds.of(5),
            //                    Degrees.of(75),
            //                    0.6,
            //                    0.2,
            //                    1280,
            //                    720,
            //                    new Translation2d(-0.3, 0),
            //                    Meters.of(0.3),
            //                    Rotation2d.fromDegrees(180),
            //                    Degrees.of(40),
            //                    Degrees.zero())
            );
    //            List.of(
    //            new PhotonCameraProperties(
    //                    "FrontCam",
    //                    30,
    //                    14,
    //                    5,
    //                    75,
    //                    0.6,
    //                    0.2,
    //                    1280,
    //                    720,
    //                    new Translation2d(
    //                            0.330, -0.127), // the outing position of the camera in relative to the robot center
    //                    0.25, // the mounting height, in meters
    //                    Rotation2d.fromDegrees(0), // the camera facing, 0 is front, positive is counter-clockwise
    //                    24, // camera pitch angle, in degrees
    //                    180 // camera roll angle, 0 for up-right and 180 for upside-down
    //                    ),
    //            new PhotonCameraProperties(
    //                    "FrontLeftCam",
    //                    30,
    //                    14,
    //                    5,
    //                    75,
    //                    0.6,
    //                    0.2,
    //                    1280,
    //                    720,
    //                    new Translation2d(0.229, 0.348),
    //                    0.2,
    //                    Rotation2d.fromDegrees(30),
    //                    30,
    //                    180 // upside-down
    //                    ),
    //            new PhotonCameraProperties(
    //                    "FrontRightCam",
    //                    30,
    //                    14,
    //                    5,
    //                    75,
    //                    0.6,
    //                    0.2,
    //                    1280,
    //                    720,
    //                    new Translation2d(0.229, -0.348),
    //                    0.2,
    //                    Rotation2d.fromDegrees(-30),
    //                    30,
    //                    180 // upside-down
    //                    ),
    //            new PhotonCameraProperties(
    //                    "BackLeftCam",
    //                    30,
    //                    14,
    //                    5,
    //                    75,
    //                    0.6,
    //                    0.2,
    //                    1280,
    //                    720,
    //                    new Translation2d(-0.229, 0.330),
    //                    0.2,
    //                    Rotation2d.fromDegrees(150),
    //                    35,
    //                    180 // upside-down
    //                    ),
    //            new PhotonCameraProperties(
    //                    "BackRightCam",
    //                    30,
    //                    14,
    //                    5,
    //                    75,
    //                    0.6,
    //                    0.2,
    //                    1280,
    //                    720,
    //                    new Translation2d(-0.229, -0.330),
    //                    0.2,
    //                    Rotation2d.fromDegrees(-150),
    //                    35,
    //                    180 // upside-down
    //                    ));
}
