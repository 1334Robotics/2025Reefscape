package frc.robot.commands.vision;

import java.util.function.DoubleUnaryOperator;

public class DoubleLookupTable {
    public final double min;
    public final double max;
    public final double space;
    public final double[] values;

    public DoubleLookupTable(double min, double max, double space, DoubleUnaryOperator function) {
        this.min = min;
        this.max = max;
        this.space = space;
        this.values = new double[(int)((max-min)/space + 0.5)];

        int index = 0;
        double i;
        for(i=min;i<max;i+=space)
            this.values[index++] = function.applyAsDouble(i);
    }

    public double get(double value) {
        if(value < this.min || value >= this.max)
            throw new IllegalArgumentException("Value out of range: " + value);

        int index = (int)((value - this.min) / this.space);
        System.out.printf(".get(%f) = %f", value, this.values[index]);
        return this.values[index];
    }
}