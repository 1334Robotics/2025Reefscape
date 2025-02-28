package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
// Remove IronMaple import
// import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.Units;

// Simulation implementation
public class GyroIOSim implements GyroIO {
    // Use a simple simulated gyro instead of IronMaple
    private Rotation2d simulatedRotation = new Rotation2d();
    private AngularVelocity simulatedVelocity = Units.RadiansPerSecond.of(0);
    
    // Default constructor with no dependencies
    public GyroIOSim() {
        System.out.println("Created simplified gyro simulation");
    }

    @Override // specified by GroIOSim interface
    public Rotation2d getGyroRotation() {
        return simulatedRotation;
    }

    @Override // specified by GroIOSim interface
    public AngularVelocity getGyroAngularVelocity() {
        return simulatedVelocity;
    }
    
    /**
     * Sets the simulated gyro rotation
     * 
     * @param rotation The rotation to set
     */
    public void setRotation(Rotation2d rotation) {
        this.simulatedRotation = rotation;
    }
    
    /**
     * Sets the simulated gyro angular velocity
     * 
     * @param velocity The angular velocity to set
     */
    public void setAngularVelocity(AngularVelocity velocity) {
        this.simulatedVelocity = velocity;
    }
}