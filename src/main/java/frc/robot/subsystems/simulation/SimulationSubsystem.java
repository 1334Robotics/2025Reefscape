package frc.robot.subsystems.simulation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeIOSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeReefSimulation;

public class SimulationSubsystem extends SubsystemBase {

    private final Field2d field = new Field2d(); // Initialize Field2d
    private final SwerveDriveSimulation swerveDriveSimulation;
    private final SwerveSubsystem swerveSubsystem;
    private final IntakeIOSim intakeIOSim;
    private final AIRobotInSimulation[] instances = new AIRobotInSimulation[1];
    

    public SimulationSubsystem(SwerveDriveSimulation swerveDriveSimulation, SwerveSubsystem swerveSubsystem) {
        this.swerveDriveSimulation = swerveDriveSimulation;
        this.swerveSubsystem = swerveSubsystem;

        // Initialize IntakeIOSim
        this.intakeIOSim = new IntakeIOSim(swerveDriveSimulation);
    }

    public void setInitialPose(Pose2d initialPose) {
        if (Robot.isSimulation()) {
            swerveDriveSimulation.setSimulationWorldPose(initialPose);
            swerveSubsystem.resetOdometry(initialPose); // Reset swerve odometry
            field.setRobotPose(initialPose);
            // Add CORAL-ALGAE stack
            SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(1.25,4)));
            SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(1.2,5.85)));
            SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(16.4,5.85)));
            SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(16.4,4)));
            SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(16.4,2.15)));
            
            SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
            // We must specify a heading since the coral is a tube
            new Pose2d(1.5, 5, Rotation2d.fromDegrees(90))));
        }
    }

    @Override
    public void periodic() {
        // Warning: DO NOT call this method on a real robot, as it can drain the roboRIO’s resources.
        SimulatedArena.getInstance().simulationPeriodic();

        // Update intake simulation
        intakeIOSim.periodic();

        Pose2d robotPose = RobotContainer.swerveSubsystem.getPose();
        Logger.recordOutput("FieldSimulation/RobotPose", new Pose3d(robotPose));

        // Log intake state
        Logger.recordOutput("Simulation/CoralInIntake", intakeIOSim.isCoralInsideIntake());

        Logger.recordOutput("FieldSimulation/Algae", 
        SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        Logger.recordOutput("FieldSimulation/Coral", 
        SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));

        Logger.recordOutput("FieldSimulation/AIRobotPoses", AIRobotInSimulation.getOpponentRobotPoses());

        Logger.recordOutput("FieldSimulation/RobotPose", swerveSubsystem.getPose());
        Logger.recordOutput("FieldSimulation/GamePieces", 
        SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    }


    // Add methods to control the intake
    public void runIntake() {
        System.out.println("DEBUG: runIntake() called");
        intakeIOSim.setRunning(true);
    }
    
    public void stopIntake() {
        intakeIOSim.setRunning(false);
    }
    
    public void launchCoral() {
        intakeIOSim.launchCoral();
    }
    
    public boolean isCoralInIntake() {
        return intakeIOSim.isCoralInsideIntake();
    }

    public Field2d getField() {
        return field;
    }
}
