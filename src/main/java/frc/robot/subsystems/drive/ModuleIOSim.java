package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.Units;

public class ModuleIOSim implements ModuleIO {
        // reference to module simulation
        private final SwerveModuleSimulation moduleSimulation;
        // reference to the simulated drive motor
        private final SimulatedMotorController.GenericMotorController driveMotor;
        // reference to the simulated turn motor
        private final SimulatedMotorController.GenericMotorController turnMotor;

    public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;

        // configures a generic motor controller for drive motor
        // set a current limit of 60 amps
        this.driveMotor = moduleSimulation
                .useGenericMotorControllerForDrive()
                .withCurrentLimit(Units.Amps.of(60));
        this.turnMotor = moduleSimulation
                .useGenericControllerForSteer()
                .withCurrentLimit(Units.Amps.of(20));
    }

    @Override
    public Rotation2d getSteerRotation() {
        return moduleSimulation.getSteerAbsoluteFacing();
    }

    @Override // specified by ModuleIO interface
    public void setDriveVoltage(Voltage voltage) {
        this.driveMotor.requestVoltage(voltage);
    }

    @Override // specified by ModuleIO interface
    public void setSteerVoltage(Voltage voltage) {
        this.turnMotor.requestVoltage(voltage);
    }

    @Override // specified by ModuleIO interface
    public Rotation2d getSteerFacing() {
        return this.moduleSimulation.getSteerAbsoluteFacing();
    }

    @Override // specified by ModuleIO interface
    public Angle getSteerRelativePosition() {
        /* 
        Divide by 12.8 to account for the total effective gear reduction between
        the motor and the steering mechanism. This ensures that the encoder value
        reflects the actual physical rotation of the steering module.
        The value 12.8 comes from combining two gear ratios, 
        which represent stages of reduction in the system.
        */
        return moduleSimulation.getSteerRelativeEncoderPosition().div(12.8);
    }

    @Override // specified by ModuleIO interface
    public Angle getDriveWheelPosition() {
        return moduleSimulation.getDriveWheelFinalPosition();
    }
}