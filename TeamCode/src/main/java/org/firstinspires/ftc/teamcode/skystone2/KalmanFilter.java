package org.firstinspires.ftc.teamcode.skystone2;

public class KalmanFilter {
    double lastValue = 0;
    double delta = 0;
    double currentValue = 0;

    void reset() {
        lastValue = 0;
        delta = 0;
        currentValue = 0;
    }

    double getEstimate() {

        return currentValue + (currentValue - lastValue) + delta;
    }

    void updateValue( double value) {
        if (value != currentValue) {
            lastValue = currentValue;
            currentValue = value;
        }
    }
}