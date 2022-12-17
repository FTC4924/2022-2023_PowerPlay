package org.firstinspires.ftc.teamcode.Commands.preftclib;

import org.firstinspires.ftc.teamcode.AutoBase;

import androidx.annotation.FloatRange;

public class Move extends Command {
    public Move(double distance, double angle, @FloatRange(from=0.0, to=1.0) double power) {
        this.distance = distance;
        this.angle = Math.toRadians(angle);
        this.power = power;
        this.constructorID = 0;
    }

    public Move(double distance, @FloatRange(from=0.0, to=1.0) double power) {
        this.distance = distance;
        this.angle = AutoBase.getTargetAngle();
        this.power = power;
        this.constructorID = 0;
    }

    public Move(double power, double angle, int timeout) {
        this.distance = 100;
        this.duration = timeout;
        this.power = power;
        this.angle = Math.toRadians(angle);
        this.constructorID = 1;
    }
}
