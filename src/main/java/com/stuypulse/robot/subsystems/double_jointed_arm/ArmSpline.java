package com.stuypulse.robot.subsystems.double_jointed_arm;

import edu.wpi.first.math.spline.QuinticHermiteSpline;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ArmSpline {
    private final QuinticHermiteSpline spline;
    private final double duration;

    public ArmSpline(double[] startTheta, double[] startVel, double[] startAccel,
                    double[] endTheta, double[] endVel, double[] endAccel,
                    double durationSec) {
        this.duration = durationSec;

        this.spline = new QuinticHermiteSpline(
            // X control vectors (theta1)
            new double[]{startTheta[0], startVel[0] * duration, startAccel[0] * duration * duration},
            new double[]{endTheta[0], endVel[0] * duration, endAccel[0] * duration * duration},
            
            // Y control vectors (theta2)
            new double[]{startTheta[1], startVel[1] * duration, startAccel[1] * duration * duration},
            new double[]{endTheta[1], endVel[1] * duration, endAccel[1] * duration * duration}
        );
    }

    public ArmTrajectoryPoint getPoint(double t) {
        double[][] allCoeffs = spline.getCoefficients().toArray2();
        int numCoeffsPerDimension = 6; // Quintic spline

        double[] coeffsX = Arrays.stream(allCoeffs)
                    .flatMapToDouble(row -> Arrays.stream(row, 0, numCoeffsPerDimension))
                    .toArray();

        double[] coeffsY = Arrays.stream(allCoeffs)
                    .flatMapToDouble(row -> Arrays.stream(row, numCoeffsPerDimension, numCoeffsPerDimension * 2))
                    .toArray();

        // Calculate position (rad)
        double theta1 = calculatePolynomial(coeffsX, t);
        double theta2 = calculatePolynomial(coeffsY, t);
        
        // Calculate velocity (rad/s)
        double omega1 = calculatePolynomialDerivative(coeffsX, t) / duration;
        double omega2 = calculatePolynomialDerivative(coeffsY, t) / duration;
        
        // Calculate acceleration (rad/s²)
        double alpha1 = calculatePolynomialSecondDerivative(coeffsX, t) / (duration * duration);
        double alpha2 = calculatePolynomialSecondDerivative(coeffsY, t) / (duration * duration);
        
        return new ArmTrajectoryPoint(
            theta1, theta2,
            omega1, omega2,
            alpha1, alpha2,
            t * duration
        );
    }

    private double calculatePolynomial(double[] coeffs, double t) {
        return coeffs[0] + coeffs[1] * t + coeffs[2] * Math.pow(t, 2)
             + coeffs[3] * Math.pow(t, 3) + coeffs[4] * Math.pow(t, 4)
             + coeffs[5] * Math.pow(t, 5);
    }

    private double calculatePolynomialDerivative(double[] coeffs, double t) {
        return coeffs[1] + 2 * coeffs[2] * t 
             + 3 * coeffs[3] * Math.pow(t, 2) + 4 * coeffs[4] * Math.pow(t, 3)
             + 5 * coeffs[5] * Math.pow(t, 4);
    }

    private double calculatePolynomialSecondDerivative(double[] coeffs, double t) {
        return 2 * coeffs[2] + 6 * coeffs[3] * t 
             + 12 * coeffs[4] * Math.pow(t, 2) + 20 * coeffs[5] * Math.pow(t, 3);
    }

    public List<ArmTrajectoryPoint> sampleTrajectory(int sampleCount) {
        List<ArmTrajectoryPoint> points = new ArrayList<>();
        for (int i = 0; i <= sampleCount; i++) {
            double t = i / (double) sampleCount;
            points.add(getPoint(t));
        }
        return points;
    }
}