package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    private final SwerveSubsystem swerve = new SwerveSubsystem();
    private final XboxController driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    private final Field2d field = new Field2d();
    private XboxControllerSim driverControllerSim;
    private boolean simulationInitialized = false;

    public RobotContainer() {
        if (RobotBase.isSimulation()) {
            initializeSimulation();
        }

        configureBindings();
        configureDefaultCommands();
        SmartDashboard.putData("Field", field);
    }

    private void initializeSimulation() {
        try {
            driverControllerSim = new XboxControllerSim(OIConstants.DRIVER_CONTROLLER_PORT);
            // Initialize all buttons and axes to prevent warnings
            for (int i = 1; i <= 10; i++) {
                driverControllerSim.setRawButton(i, false);
            }
            driverControllerSim.setLeftX(0);
            driverControllerSim.setLeftY(0);
            driverControllerSim.setRightX(0);
            driverControllerSim.setRightY(0);
            simulationInitialized = true;
        } catch (Exception e) {
            System.err.println("Failed to initialize simulation controller: " + e.getMessage());
        }
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(
            new SwerveDriveCommand(
                swerve,
                driverController,
                () -> true
            )
        );
    }

    private void configureBindings() {
        // Reset gyro with Y button
        if (driverController.getYButton()) {
            swerve.zeroHeading();
        }
        
        // Lock wheels in X pattern with B button
        if (driverController.getBButton()) {
            swerve.drive(0, 0, 0, true);
        }
        
        // Toggle field relative/robot relative with X button
        if (driverController.getXButton()) {
            swerve.toggleFieldRelative();
        }
        
        // Slow mode while holding right bumper
        if (driverController.getRightBumper()) {
            swerve.setMaxSpeed(OIConstants.SLOW_MODE_SPEED);
        } else {
            swerve.setMaxSpeed(OIConstants.MAX_SPEED);
        }
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }

    public void simulationPeriodic() {
        if (simulationInitialized && driverControllerSim != null) {
            updateSimulatedInputs();
            driverControllerSim.notifyNewData();
        }
        field.setRobotPose(swerve.getPose());
    }

    private void updateSimulatedInputs() {
        // Update simulation inputs here if needed
        // This is where you would handle keyboard input for testing
    }
}