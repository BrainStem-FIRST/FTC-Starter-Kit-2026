package org.firstinspires.ftc.teamcode.utils.misc;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class BatteryVoltageFilter {
    public static final double initialVoltageGuess = 13;
    public static final double alpha = .8;

    public static BatteryVoltageFilter getInstance(HardwareMap hardwareMap) {
        return new BatteryVoltageFilter(hardwareMap);
    }

    private final VoltageSensor voltageSensor;

    private double voltage;

    public BatteryVoltageFilter(HardwareMap hardwareMap) {
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = initialVoltageGuess;
    }

    public void update() {
        double rawVoltage = voltageSensor.getVoltage();
        voltage = voltage * alpha + rawVoltage * (1 - alpha);
    }
    public double getVoltage() {
        return voltage;
    }
}
