package frc.robot.subsystems.intake;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import frc.robot.constants.SimulationConstants;

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
        SimulationConstants.INTAKE_LENGTH,
        // The extension length of the intake beyond the robot's frame (when activated)
        SimulationConstants.INTAKE_EXTENSION,
        // The intake is mounted on the back side of the chassis
        IntakeSimulation.IntakeSide.BACK,
        // The intake can hold up to 1 note
        1);
    }

    public void periodic() {
        // Add any periodic updates for the intake simulation here
        // For example, you might want to update the intake's position based on the robot's movement
    }    

    @Override
    public void setRunning(boolean runIntake) {
        if (runIntake) {
            intakeSimulation.startIntake(); // Extends the intake out from the chassis frame and starts detecting contacts with game pieces
        } else {
            intakeSimulation.stopIntake(); // Retracts the intake into the chassis frame, disabling game piece collection
        }
    }

    @Override
    public boolean isNoteInsideIntake() {
        return intakeSimulation.getGamePiecesAmount() != 0; // True if there is a game piece in the intake
    }

    @Override
    public void launchNote() {
        // if there is a note in the intake, it will be removed and return true; otherwise, returns false
        if (intakeSimulation.obtainGamePieceFromIntake()) {
            // L3
            SimulatedArena.getInstance()
    .addGamePieceProjectile(new ReefscapeCoralOnFly(
        // Obtain robot position from drive simulation
        swerveDriveSimulation.getSimulatedDriveTrainPose().getTranslation(),
        // The scoring mechanism is installed at (0.35, 0) (meters) on the robot
        new Translation2d(SimulationConstants.SCORING_MECH_L3_HEIGHT, 0),
        // Obtain robot speed from drive simulation
        swerveDriveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        // Obtain robot facing from drive simulation
        swerveDriveSimulation.getSimulatedDriveTrainPose().getRotation(),
        // The height at which the coral is ejected
        SimulationConstants.L3_BRANCH_HEIGHT,
        // The initial speed of the coral
        Units.MetersPerSecond.of(2),
        // The coral is ejected at a 35-degree slope
        Units.Degrees.of(-35)));
        }
    }
}
