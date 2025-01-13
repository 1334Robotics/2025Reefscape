// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDsubsystem{
    private double kP, kI, kD;
    private double tau;
    private double limMin, limMax;
    private double limMinInt, limMaxInt;
    private double sampleTime;
    private double integrator, differentiator;
    private double prevError, prevMeasurement;
    private double proportional;

    public PID(double kP, double kI, double kD, double tau,
               double limMin, double limMax, double limMinInt, double limMaxInt, double sampleTime) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.tau = tau;
        this.limMin = limMin;
        this.limMax = limMax;
        this.limMinInt = limMinInt;
        this.limMaxInt = limMaxInt;
        this.sampleTime = sampleTime;

        // Reset some values
        this.integrator = 0;
        this.prevError = 0;
        this.prevMeasurement = 0;
        this.differentiator = 0;
    }

    // Updates the PID controller with the given error
    public void update(double setPoint, double measurement) {
        double error = setPoint - measurement;
        // Calculate the proportional term
        this.proportional = this.kP * error;

        // Calculate and clamp the integrator
        this.integrator = this.integrator + (0.5 * this.kI * this.sampleTime * (error + this.prevError));
        if(this.integrator > this.limMaxInt) this.integrator = this.limMaxInt;
        if(this.integrator < this.limMinInt) this.integrator = this.limMinInt;

        // Differentiate
        this.differentiator = -(2 * this.kD * (measurement - this.prevMeasurement)
                + (2 * this.tau - this.sampleTime) * this.differentiator)
                / (2 * this.tau + this.sampleTime);

        this.prevError = error;
        this.prevMeasurement = measurement;
    }

    // Gets the rotational steer value for the robot
    public double getSteer() {
        double output = this.proportional + this.integrator + this.differentiator;
        if(output > this.limMax) output = this.limMax;
        if(output < this.limMin) output = this.limMin;

        return output;
    }

}
