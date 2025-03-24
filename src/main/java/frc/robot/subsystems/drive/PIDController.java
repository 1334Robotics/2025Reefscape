package frc.robot.subsystems.drive;

public class PIDController {
    private final double kP, kI, kD;
    private final double tau;
    private final double limMin, limMax;
    private final double limMinInt, limMaxInt;
    private final double sampleTime;
    private double proportionalTerm, integralTerm, differentialTerm;
    private double prevError, prevMeasurement;
    private double prevOutput;

    public PIDController(double kP, double kI, double kD, double tau, double limMin,
               double limMax, double limMinInt, double limMaxInt, double sampleTime) {
        this.kP         = kP;
        this.kI         = kI;
        this.kD         = kD;
        this.tau        = tau;
        this.limMin     = limMin;
        this.limMax     = limMax;
        this.limMinInt  = limMinInt;
        this.limMaxInt  = limMaxInt;
        this.sampleTime = sampleTime;

        // Set default values
        this.proportionalTerm = 0;
        this.integralTerm     = 0;
        this.differentialTerm = 0;
        this.prevError        = 0;
        this.prevMeasurement  = 0;
        this.prevOutput       = 0;
    }

    public void update(double target, double measurement) {
        double error = target - measurement;
        
        // Calculate the proportional term
        this.proportionalTerm = this.kP * error;

        // Calculate and clamp the integral term
        this.integralTerm += 0.5 * this.kI * this.sampleTime * (error + this.prevError);
        if(this.integralTerm > this.limMaxInt) this.integralTerm = this.limMaxInt;
        if(this.integralTerm < this.limMinInt) this.integralTerm = this.limMinInt;

        // Differentiate
        this.differentialTerm = -(2 * this.kD * (measurement - this.prevMeasurement)
                                + (2 * this.tau - this.sampleTime) * this.differentialTerm)
                                / (2 * this.tau + this.sampleTime);
        
        // Set the previous values
        this.prevError       = error;
        this.prevMeasurement = measurement;
    }

    public double getOutput() {
        double output = this.proportionalTerm + this.integralTerm + this.differentialTerm;
        if(output > this.limMax) output = this.limMax;
        if(output < this.limMin) output = this.limMin;

        this.prevOutput = output;
        return output;
    }

    public double getPreviousOutput() {
        return this.prevOutput;
    }

    public void zero() {
        this.prevError        = 0;
        this.prevMeasurement  = 0;
        this.proportionalTerm = 0;
        this.integralTerm     = 0;
        this.differentialTerm = 0;
        this.prevOutput       = 0;
    }
}