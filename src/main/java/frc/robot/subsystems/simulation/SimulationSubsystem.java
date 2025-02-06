package frc.robot.subsystems.simulation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import org.littletonrobotics.junction.Logger; // AdvantageKit logging
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

public class SimulationSubsystem extends SubsystemBase {

    public SimulationSubsystem() {
        
    }

    @Override
    public void periodic() {
        //🔴 Warning: DO NOT call this method on a real robot, as it can drain the roboRIO’s resources.
        SimulatedArena.getInstance().simulationPeriodic();

        Pose2d robotPose = RobotContainer.swerveSubsystem.getPose();
        Logger.recordOutput("FieldSimulation/RobotPose", new Pose3d(robotPose));
        // Get the positions of the notes (both on the field and in the air)
        Pose3d[] notesPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Note");

        // Publish to telemetry using AdvantageKit
        Logger.recordOutput("FieldSimulation/NotesPositions", notesPoses);
    }

        public void addGamePiece(Translation2d position) {
            SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(new Pose2d(2, 2, Rotation2d.fromDegrees(90))));
            SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(3,2)));
            SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(4,4)));
    }

    public void clearGamePieces() {
        SimulatedArena.getInstance().clearGamePieces();
    }
}
