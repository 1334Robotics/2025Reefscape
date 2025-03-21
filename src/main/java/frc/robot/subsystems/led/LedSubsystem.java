package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    // Digital output to control the LED
    private final DigitalOutput led;

    public LedSubsystem(int channel) {
        // Initialize the digital output on the specified channel.
        led = new DigitalOutput(channel);
    }

    /** Turns the LED on. */
    public void turnOn() {
        led.set(true);
    }

    /** Turns the LED off. */
    public void turnOff() {
        led.set(false);
    }


    public void toggle() {
        led.set(!led.get());
    }

    @Override
    public void periodic() {
    }
}