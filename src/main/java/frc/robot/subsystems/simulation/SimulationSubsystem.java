package frc.robot.subsystems.simulation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeIOSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.littletonrobotics.junction.Logger;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;

public class SimulationSubsystem extends SubsystemBase {

    private final Field2d field = new Field2d(); // Initialize Field2d
    private final SwerveDriveSimulation swerveDriveSimulation;
    private final SwerveSubsystem swerveSubsystem;
    private final IntakeIOSim intakeIOSim;
    

    public SimulationSubsystem(SwerveDriveSimulation swerveDriveSimulation, SwerveSubsystem swerveSubsystem, IntakeIOSim intakeIOSim) {
        this.swerveDriveSimulation = swerveDriveSimulation;
        this.swerveSubsystem = swerveSubsystem;
        this.intakeIOSim = intakeIOSim;
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
        }
    }

    @Override
    public void periodic() {
        // Warning: DO NOT call this method on a real robot, as it can drain the roboRIO’s resources.
        SimulatedArena.getInstance().simulationPeriodic();

        // Log game piece positions
        Logger.recordOutput("FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        Logger.recordOutput("FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));

        // Log robot component positions
        Logger.recordOutput("Robot/ElevatorHeight", RobotContainer.elevatorSubsystem.getCurrentHeight());
        Logger.recordOutput("Robot/IntakeState", RobotContainer.intakeSubsystem.isCoralInsideIntake() ? 1 : 0);

        // Update intake simulation
        intakeIOSim.periodic();

        // Log intake state
        Logger.recordOutput("Simulation/CoralInIntake", intakeIOSim.isCoralInsideIntake());

        Logger.recordOutput("FieldSimulation/AIRobotPoses", AIRobotInSimulation.getOpponentRobotPoses());

        Logger.recordOutput("FieldSimulation/RobotPose", swerveSubsystem.getPose());
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
