package org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.buildingBlocks;

import org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.tolerance.CircleTolerance;
import org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.tolerance.Tolerance;

/**
 * Container class for all configurable Bezier path parameters.
 */
public class BezierParams {

    public double speedKp = 1.0;
    public double speedKf = 0.1;
    public double headingKp = 0.3;
    public double headingKf = 0.01;
    public double centripetalForceGain = 1;

    public Tolerance tolerance = new CircleTolerance();

    public boolean passPosition = false;

    public double minLinearSpeed = 0.0;
    public double maxLinearSpeed = 100.0;
    public double maxDrivePowerRampRate = 100.0;
    public double maxTurnPower = 1.0;
    public double maxTurnPowerRampRate = 100.0;
    public double correctivePower = 0.7;
    public double maxAcceleration = 75.0;

    public double maxTime = 10.0;

    public BezierParams setSpeedKp(double speedKp) {
        this.speedKp = speedKp;
        return this;
    }

    public BezierParams setSpeedKf(double speedKf) {
        this.speedKf = speedKf;
        return this;
    }

    public BezierParams setHeadingKp(double headingKp) {
        this.headingKp = headingKp;
        return this;
    }

    public BezierParams setHeadingKf(double headingKf) {
        this.headingKf = headingKf;
        return this;
    }

    public BezierParams setTolerance(Tolerance tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public BezierParams setPassPosition(boolean passPosition) {
        this.passPosition = passPosition;
        return this;
    }

    public BezierParams setMinLinearSpeed(double minLinearPower) {
        this.minLinearSpeed = minLinearPower;
        return this;
    }

    public BezierParams setMaxLinearSpeed(double maxLinearSpeed) {
        this.maxLinearSpeed = maxLinearSpeed;
        return this;
    }

    public BezierParams setFixedLinearPower(double power) {
        this.minLinearSpeed = power;
        this.maxLinearSpeed = power;
        return this;
    }

    public BezierParams setMaxDrivePowerRampRate(double maxDrivePowerRampRate) {
        this.maxDrivePowerRampRate = maxDrivePowerRampRate;
        return this;
    }

    public BezierParams setMaxTurnPower(double maxTurnPower) {
        this.maxTurnPower = maxTurnPower;
        return this;
    }

    public BezierParams setMaxTurnPowerRampRate(double maxTurnPowerRampRate) {
        this.maxTurnPowerRampRate = maxTurnPowerRampRate;
        return this;
    }

    public BezierParams setCorrectivePower(double correctivePower) {
        this.correctivePower = correctivePower;
        return this;
    }

    public BezierParams setMaxTime(double maxTime) {
        this.maxTime = maxTime;
        return this;
    }

    public BezierParams setMaxAcceleration(double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
        return this;
    }
}
