package frc.robot.subsystems.simulation.intake;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;

public class IntakeIOSim implements IntakeIO{
    private final IntakeSimulation intakeSimulation;
    private final SwerveDriveSimulation swerveDriveSimulation;

    public IntakeIOSim(SwerveDriveSimulation swerveDriveSimulation) {
        this.swerveDriveSimulation = swerveDriveSimulation;
        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
        // Specify the type of game pieces that the intake can collect
        "Coral",
        // Specify the drivetrain to which this intake is attached
        swerveDriveSimulation,
        // Width of the intake
        Units.Meters.of(0.4),
        // The extension length of the intake beyond the robot's frame (when activated)
        Units.Meters.of(0.2),
        // The intake is mounted on the back side of the chassis
        IntakeSimulation.IntakeSide.BACK,
        // The intake can hold up to 1 note
        1);
    }

        @Override
    public void setRunning(boolean runIntake) {
        if (runIntake) {
            intakeSimulation.startIntake();
        } else {
            intakeSimulation.stopIntake();
        }
    }

    @Override
    public boolean isNoteInsideIntake() {
        return intakeSimulation.getGamePiecesAmount() != 0;
    }

    @Override
    public void launchNote() {
        if (intakeSimulation.obtainGamePieceFromIntake()) {
            SimulatedArena.getInstance()
    .addGamePieceProjectile(new ReefscapeCoralOnFly(
        // Obtain robot position from drive simulation
        swerveDriveSimulation.getSimulatedDriveTrainPose().getTranslation(),
        // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
        new Translation2d(0.35, 0),
        // Obtain robot speed from drive simulation
        swerveDriveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        // Obtain robot facing from drive simulation
        swerveDriveSimulation.getSimulatedDriveTrainPose().getRotation(),
        // The height at which the coral is ejected
        Units.Meters.of(1.28),
        // The initial speed of the coral
        Units.MetersPerSecond.of(2),
        // The coral is ejected at a 35-degree slope
        Units.Degrees.of(-35)));

        SimulatedArena.getInstance()
    .addGamePieceProjectile(new ReefscapeCoralOnFly(
        // Obtain robot position from drive simulation
        swerveDriveSimulation.getSimulatedDriveTrainPose().getTranslation(),
        // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
        new Translation2d(0.46, 0),
        // Obtain robot speed from drive simulation
        swerveDriveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        // Obtain robot facing from drive simulation
        swerveDriveSimulation.getSimulatedDriveTrainPose().getRotation(),
        // The height at which the coral is ejected
        Units.Meters.of(2.1),
        // The initial speed of the coral
        Units.MetersPerSecond.of(1),
        // The coral is ejected vertically downwards
        Units.Degrees.of(-90)));

        }
    }
}
