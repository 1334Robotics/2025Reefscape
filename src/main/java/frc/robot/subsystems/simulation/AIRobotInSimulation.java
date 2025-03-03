package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.Arrays;
import java.util.Objects;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.utils.FieldMirroringUtils;

/**
 *
 *
 * <h2>Represents an AI robot during simulation.</h2>
 *
 * <p>This class models an AI-controlled robot used during simulation for driver practice and skill development.
 *
 * <p>The AI robots are capable of performing various tasks to enhance training scenarios, including:
 *
 * <ul>
 *   <li>Automatically cycling around the field to help practice offensive skills.
 *   <li>Delivering feed-shots to assist in practicing front-field clean-ups.
 *   <li>Driving via joystick control to simulate defense and counter-defense scenarios.
 * </ul>
 */
public class AIRobotInSimulation extends SubsystemBase {
    /* If an opponent robot is not requested to be on the field, it is placed ("queens") outside the field at predefined positions. */
    public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
        new Pose2d(-6, 0, new Rotation2d()),
        new Pose2d(-5, 0, new Rotation2d()),
        new Pose2d(-4, 0, new Rotation2d()),
        new Pose2d(-3, 0, new Rotation2d()),
        new Pose2d(-2, 0, new Rotation2d())
    };
    /* The robots will be teleported to these positions when teleop begins. */
    public static final Pose2d[] ROBOTS_STARTING_POSITIONS = new Pose2d[] {
        new Pose2d(15, 6, Rotation2d.fromDegrees(180)),
        new Pose2d(15, 4, Rotation2d.fromDegrees(180)),
        new Pose2d(15, 2, Rotation2d.fromDegrees(180)),
        new Pose2d(1.6, 6, new Rotation2d()),
        new Pose2d(1.6, 4, new Rotation2d())
    };
    /* Store instances of AI robots in a static array. */
    public static final AIRobotInSimulation[] instances = new AIRobotInSimulation[6];
    SendableChooser<Command> behaviorChooser = new SendableChooser<>();

    /* The drivetrain configuration for the opponent robots in the maple-sim simulation. */
    private static final DriveTrainSimulationConfig DRIVETRAIN_CONFIG =
            DriveTrainSimulationConfig.Default().withRobotMass(Kilograms.of(45));

    /* The PathPlanner configuration for the opponent robots. */
    private static final RobotConfig PP_CONFIG = new RobotConfig(
            Kilograms.of(55),
            KilogramSquareMeters.of(6),
            new ModuleConfig(
                    Units.inchesToMeters(2), 3.5, 1.2, DCMotor.getFalcon500(1).withReduction(8.14), 60, 1),
            Meters.of(0.6));
    private static final PPHolonomicDriveController driveController =
            new PPHolonomicDriveController(new PIDConstants(5.0, 0.02), new PIDConstants(7.0, 0.05));

    /**
     *
     *
     * <h2>Activates the opponent robots.</h2>
     *
     * <p>The opponent robots <strong>will not</strong> appear on the field immediately. They are initially placed at
     * the {@link #ROBOT_QUEENING_POSITIONS}.
     *
     * <p>Instead of being active right away, a sendable chooser is sent to the dashboard, allowing the user to select
     * the mode of these robots.
     */
    public static void startOpponentRobotSimulations() {
        try {
            // Creates an instance of the first AI robot
            instances[0] = new AIRobotInSimulation(0);
            // Builds the behavior chooser for the first AI robot
            instances[0].buildBehaviorChooser(
                    PathPlannerPath.fromPathFile("opponent robot cycle path 0"),
                    Commands.none(),
                    PathPlannerPath.fromPathFile("opponent robot cycle path 0 backwards"),
                    Commands.none(),
                    new XboxController(2));

            // Same of the following:

            instances[1] = new AIRobotInSimulation(1);
            instances[1].buildBehaviorChooser(
                    PathPlannerPath.fromPathFile("opponent robot cycle path 1"),
                    Commands.none(),
                    PathPlannerPath.fromPathFile("opponent robot cycle path 1 backwards"),
                    Commands.none(),
                    new XboxController(3));

            instances[2] = new AIRobotInSimulation(2);
            instances[2].buildBehaviorChooser(
                    PathPlannerPath.fromPathFile("opponent robot cycle path 2"),
                    Commands.none(),
                    PathPlannerPath.fromPathFile("opponent robot cycle path 2 backwards"),
                    Commands.none(),
                    new XboxController(4));

            instances[3] = new AIRobotInSimulation(3);
            instances[3].buildBehaviorChooser(
                    PathPlannerPath.fromPathFile("opponent robot cycle path 3"),
                    Commands.none(),
                    PathPlannerPath.fromPathFile("opponent robot cycle path 3 backwards"),
                    Commands.none(),
                    new XboxController(5));

            instances[4] = new AIRobotInSimulation(4);
            instances[4].buildBehaviorChooser(
                    PathPlannerPath.fromPathFile("opponent robot cycle path 4"),
                    Commands.none(),
                    PathPlannerPath.fromPathFile("opponent robot cycle path 4 backwards"),
                    Commands.none(),
                    new XboxController(6));

                instances[5] = new AIRobotInSimulation(5);
            instances[5].buildBehaviorChooser(
                    PathPlannerPath.fromPathFile("opponent robot cycle path 5"),
                    Commands.none(),
                    PathPlannerPath.fromPathFile("opponent robot cycle path 5 backwards"),
                    Commands.none(),
                    new XboxController(7));

        } catch (Exception e) {
            DriverStation.reportError("failed to load opponent robot simulation path, error:" + e.getMessage(), false);
        }
    }

    private final SelfControlledSwerveDriveSimulation driveSimulation;
    private final Pose2d queeningPose;
    private final int id;

    public AIRobotInSimulation(int id) {
        System.out.println("DEBUG: Initializing AI robot " + id);
        this.id = id;
        this.queeningPose = ROBOT_QUEENING_POSITIONS[id];
        this.driveSimulation =
                new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(DRIVETRAIN_CONFIG, queeningPose));

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());
    }

    /**
     * Builds the behavior chooser for this opponent robot and sends it to the dashboard. This allows the user to select
     * the robot's behavior mode during simulation.
     */
    public void buildBehaviorChooser(
        PathPlannerPath segment0,
        Command toRunAtEndOfSegment0,
        PathPlannerPath segment1,
        Command toRunAtEndOfSegment1,
        XboxController joystick) {

        // Supplier to disable the robot: sets the robot's pose to the queening position and stops its movement
        final Supplier<Command> disable =
                () -> Commands.runOnce(() -> driveSimulation.setSimulationWorldPose(queeningPose), this)
                        .andThen(Commands.runOnce(() -> driveSimulation.runChassisSpeeds(
                                new ChassisSpeeds(), new Translation2d(), false, false)))
                        .ignoringDisable(true);

        // The option to disable the robot
        behaviorChooser.setDefaultOption("Disable", disable.get());

        // The option for the robot to automatically cycle around the field
        behaviorChooser.addOption(
                "Auto Cycle", getAutoCycleCommand(segment0, toRunAtEndOfSegment0, segment1, toRunAtEndOfSegment1));

        // The option for manual joystick control of the robot
        behaviorChooser.addOption("Joystick Drive", joystickDrive(joystick));

        // Schedule the selected command when another behavior is chosen
        behaviorChooser.onChange((Command::schedule));

        // Schedule the command when teleop mode is enabled
        RobotModeTriggers.teleop()
                .onTrue(Commands.runOnce(() -> behaviorChooser.getSelected().schedule()));

        // Disable the robot when the user robot is disabled
        RobotModeTriggers.disabled().onTrue(disable.get());

        System.out.println("DEBUG: Building behavior chooser for AI robot " + id);

        // Display the behavior chooser on the dashboard for the user to select the desired robot behavior
        SmartDashboard.putData("AIRobotBehaviors/Opponent Robot " + id + " Behavior", behaviorChooser);

        // Explicitly add the SendableChooser to the AIRobotBehaviors tab in Shuffleboard
        Shuffleboard.getTab("AIRobotBehaviors")
                .add("Opponent Robot " + id + " Behavior", behaviorChooser)
                .withPosition(0, id) // Adjust position as needed
                .withSize(2, 1); // Adjust size as needed
    }

    /**
     * Retrieves the command that drives the robot to automatically cycle around the field. The robot's movement is
     * based on a relative path for auto-cycling.
     */
    private Command getAutoCycleCommand(
            PathPlannerPath segment0,
            Command toRunAtEndOfSegment0,
            PathPlannerPath segment1,
            Command toRunAtEndOfSegment1) {
        final SequentialCommandGroup cycle = new SequentialCommandGroup();
        final Pose2d startingPose = new Pose2d(
                segment0.getStartingDifferentialPose().getTranslation(),
                segment0.getIdealStartingState().rotation());
        cycle.addCommands(
                opponentRobotFollowPath(segment0).andThen(toRunAtEndOfSegment0).withTimeout(10));
        cycle.addCommands(
                opponentRobotFollowPath(segment1).andThen(toRunAtEndOfSegment1).withTimeout(10));

        return cycle.repeatedly()
                .beforeStarting(Commands.runOnce(() -> driveSimulation.setSimulationWorldPose(
                        FieldMirroringUtils.toCurrentAlliancePose(startingPose))));
    }

    /**
     * Retrieves the command that drives the robot to automatically cycle around the field. The robot's movement is
     * based on a relative path for auto-cycling.
     */
    private Command opponentRobotFollowPath(PathPlannerPath path) {
        return new FollowPathCommand(
                path, // Specify the path the opponent robot should follow
                // Provide PathPlanner with the actual robot pose in the simulation, ignoring odometry error
                driveSimulation::getActualPoseInSimulationWorld,
                // Provide PathPlanner with the actual robot speed in the simulation, ignoring encoder measurement error
                driveSimulation::getActualSpeedsRobotRelative,
                // Output chassis speeds to control the robot's movement
                (speeds, feedforwards) -> driveSimulation.runChassisSpeeds(speeds, new Translation2d(), false, false),
                driveController, // Specify the PID controller for path following
                PP_CONFIG, // Provide the robot's configuration for accurate simulation
                // Flip path based on alliance side (Red vs. Blue)
                () -> DriverStation.getAlliance()
                        .orElse(DriverStation.Alliance.Blue)
                        .equals(DriverStation.Alliance.Red),
                this // AIRobotInSimulation is a subsystem, so the command uses it as a requirement
                );
    }

    /**
     * Joystick drive command for controlling the opponent robots. This command allows the robot to be driven using an
     * Xbox controller.
     */
    private Command joystickDrive(XboxController joystick) {
        // Obtain chassis speeds from the joystick inputs
        final Supplier<ChassisSpeeds> joystickSpeeds = () -> new ChassisSpeeds(
                -joystick.getLeftY() * driveSimulation.maxLinearVelocity().in(MetersPerSecond), // Forward/Backward
                -joystick.getLeftX() * driveSimulation.maxLinearVelocity().in(MetersPerSecond), // Left/Right
                -joystick.getRightX() * driveSimulation.maxAngularVelocity().in(RadiansPerSecond) // Rotation
                );

        // Obtain the driver station facing for the opponent alliance
        // Used in defense practice, where two tabs of AScope show the driver stations of both alliances
        final Supplier<Rotation2d> opponentDriverStationFacing = () ->
                FieldMirroringUtils.getCurrentAllianceDriverStationFacing().plus(Rotation2d.fromDegrees(180));

        return Commands.run(
                        () -> {
                            // Calculate field-centric speed from the driver station-centric speed
                            final ChassisSpeeds fieldCentricSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                                    joystickSpeeds.get(),
                                    FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
                                            .plus(Rotation2d.fromDegrees(180)));
                            // Run the field-centric speed to control the robot's movement
                            driveSimulation.runChassisSpeeds(fieldCentricSpeeds, new Translation2d(), true, true);
                        },
                        this)
                // Before the command starts, reset the robot to its starting position on the field
                .beforeStarting(() -> driveSimulation.setSimulationWorldPose(
                        FieldMirroringUtils.toCurrentAlliancePose(ROBOTS_STARTING_POSITIONS[id - 1])));
    }

    public static Pose2d[] getOpponentRobotPoses() {
        return Arrays.stream(instances)
            .filter(Objects::nonNull) // Filter out null instances
            .map(instance -> instance.driveSimulation.getActualPoseInSimulationWorld())
            .toArray(Pose2d[]::new);
    }

    public static Pose2d[] getAlliancePartnerRobotPoses() {
        return getRobotPoses(new AIRobotInSimulation[] {instances[3], instances[4]});
    }

    public static Pose2d[] getRobotPoses(AIRobotInSimulation[] instances) {
        return Arrays.stream(instances)
                .filter(Objects::nonNull) // Filter out null instances
                .map(instance -> instance.driveSimulation.getActualPoseInSimulationWorld())
                .toArray(Pose2d[]::new);
        }
}