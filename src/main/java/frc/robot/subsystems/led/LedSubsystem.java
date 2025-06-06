package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private final int length = 60; // Number of LEDs in strip
    private boolean isOn = false;

    public LedSubsystem(int channel) {
        led = new AddressableLED(channel);
        ledBuffer = new AddressableLEDBuffer(length);
        led.setLength(length);
        
        // Initialize LED strip
        led.setData(ledBuffer);
        led.start();
    }

    /** Sets all LEDs to a specific RGB color */
    public void setColour(int r, int g, int b) {
        for (int i = 0; i < length; i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        led.setData(ledBuffer);
        isOn = true;
    }

    /** Turns all LEDs off */
    public void turnOff() {
        setColour(0, 0, 0);
        isOn = false;
    }

    /** Turns LEDs on to last color (or white if no color set) */
    public void turnOn() {
        if(!isOn) this.setColour(255, 255, 255);
    }

    /** Toggles between on and off */
    public void toggle() {
        if (isOn) {
            turnOff();
        } else {
            turnOn();
        }
    }

    @Override
    public void periodic() {
        // Periodic code here if needed
    }
}