package org.firstinspires.ftc.teamcode.roadrunner.util.math;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;

public class TimeManager {
    private ArrayList<Double> times;
    private ArrayList<TimeHandler> timeHandlers;
    private double time;
    private double timeDelta;
    private OpMode opMode;

    public TimeManager() {
        this.times = new ArrayList<Double>();
        this.timeHandlers = new ArrayList<TimeHandler>();
    }

    public static interface TimeHandler {
        public void execute();
    }

    public TimeManager setOpMode(OpMode opMode) {
        this.opMode = opMode;
        return this;
    }

    public void subscribeTimeEvent(double time, TimeHandler timeHandler) {
        this.times.add(this.time + time);
        this.timeHandlers.add(timeHandler);
    }

    public OpMode getOpMode() {
        return this.opMode;
    }

    public double getTime() {
        return this.time;
    }

    public double getTimeDelta() {
        return this.timeDelta;
    }

    public void update() {
        double nowTime = opMode.time;
        for (int i = timeHandlers.size() - 1; i >= 0; i--) {
            double time = this.times.get(i);
            TimeHandler timeHandler = this.timeHandlers.get(i);
            if (time < nowTime){
                timeHandler.execute();
                this.times.remove(i);
                this.timeHandlers.remove(i);
            }
        }
        this.timeDelta = nowTime - this.time;
        this.time = nowTime;
    }
}