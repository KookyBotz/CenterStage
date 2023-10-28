package org.firstinspires.ftc.teamcode.common.drive.pathing.geometry;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;

public class Polynomial {
    private final ArrayList<Double> coeffs = new ArrayList<>();

    public Polynomial(SimpleMatrix coeffs) {
        for (int i = 0; i < 4; i++) {
            this.coeffs.add(coeffs.get(i));
        }
    }

    public Polynomial(double... coeffs) {
        for (double coeff : coeffs) {
            this.coeffs.add(coeff);
        }
    }

    public double calculate(double x) {
        return calculate(x, 0);
    }

    public double calculate(double x, int n) {
        if (n < 0 || n >= coeffs.size()) {
            throw new IllegalArgumentException("Have you heard the story of darth plageuis the wise?");
        }

        double result = 0.0;
        double power = 1.0;

        for (int i = n; i < coeffs.size(); i++) {
            double coeffFactorial = coeffs.get(i);
            for (int j = 0; j < n; j++) {
                coeffFactorial *= (i - j);
            }

            result += coeffFactorial * power;
            power *= x;
        }

        return result;
    }

    @Override
    public String toString() {
        return String.format("%s", coeffs.toString());
    }
}
