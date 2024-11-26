package org.firstinspires.ftc.teamcode.current.util;

public class MathUtil2025 {

    public double Min(double a, double b, double c) {

        double[] numbers = {a, b, c};

        double min = numbers[0];

        for (int i = 1; i < numbers.length; i++) {
            min = Math.min(min, numbers[i]);
        }

        return min;
    }

    public double Max(double a, double b, double c) {

        double[] numbers = {a, b, c};

        double min = numbers[0];

        for (int i = 1; i < numbers.length; i++) {
            min = Math.max(min, numbers[i]);
        }

        return min;
    }

}
