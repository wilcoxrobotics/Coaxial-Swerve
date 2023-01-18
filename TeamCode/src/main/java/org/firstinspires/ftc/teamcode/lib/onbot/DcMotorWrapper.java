package org.firstinspires.ftc.teamcode.lib.onbot;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.roadrunner.util.math.MathUtil;
import org.firstinspires.ftc.teamcode.roadrunner.util.math.TimeManager;

public class DcMotorWrapper {
    private static TimeManager timeManager;

    private DcMotor _dcMotor;
    private int _position = 0;
    private int _targetPosition = 0;
    private int _lowerBound = 0;
    private int _upperBound = 1;
    private double _forwardPower = 0.5;
    private double _reversePower = 0.5;
    private boolean _isBusy = false;
    private boolean _isAsync = false;
    private EventManager becomeBusyEventManager;
    private EventManager becomeIdleEventManager;

    public DcMotorWrapper() {
        this.becomeBusyEventManager = new EventManager();
        this.becomeIdleEventManager = new EventManager();
    }

    public static void setTimeManager(TimeManager timeManager) {
        DcMotorWrapper.timeManager = timeManager;
    }

    public DcMotorWrapper setDcMotor(DcMotor dcMotor, boolean usingEncoder) {
        this._dcMotor = dcMotor;
        if (usingEncoder) {
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    public DcMotorWrapper setDcMotor(DcMotor dcMotor) {
        return this.setDcMotor(dcMotor, true);
    }

    public DcMotorWrapper setLowerBound(int lowerBound) {
        this._lowerBound = lowerBound;
        return this;
    }

    public DcMotorWrapper setUpperBound(int upperBound) {
        this._upperBound = upperBound;
        return this;
    }

    public DcMotorWrapper setRange(int lowerBound, int upperBound) {
        this._lowerBound = lowerBound;
        this._upperBound = upperBound;
        return this;
    }

    public DcMotorWrapper setForwardPower(double forwardPower) {
        this._forwardPower = forwardPower;
        return this;
    }

    public DcMotorWrapper setReversePower(double reversePower) {
        this._reversePower = reversePower;
        return this;
    }

    public DcMotorWrapper setPowers(double forwardPower, double reversePower) {
        this._forwardPower = forwardPower;
        this._reversePower = reversePower;
        return this;
    }

    public DcMotorWrapper setPower(double power) {
        this._forwardPower = power;
        this._reversePower = power;
        return this;
    }

    public DcMotorWrapper setPosition(double weight) {
        int targetPosition = MathUtil.applyWeight(this._lowerBound, this._upperBound, weight);
        double power = 0.0;
        if (targetPosition > this._position) power = this._forwardPower;
        if (targetPosition < this._position) power = this._reversePower;
        this._targetPosition = targetPosition;
        this._dcMotor.setTargetPosition(targetPosition);
        this._dcMotor.setPower(power);
        this._dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return this;
    }

    public DcMotorWrapper setPosition(int targetPosition) {
        double power = 0.0;
        if (targetPosition > this._position) power = this._forwardPower;
        if (targetPosition < this._position) power = this._reversePower;
        this._dcMotor.setTargetPosition(targetPosition);
        this._dcMotor.setPower(power);
        this._dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return this;
    }

    public DcMotorWrapper subscribeBecomeBusyEvent(EventManager.EventHandler eventHandler) {
        this.becomeBusyEventManager.subscribe(eventHandler);
        return this;
    }

    public DcMotorWrapper subscribeBecomeIdleEvent(EventManager.EventHandler eventHandler) {
        this.becomeIdleEventManager.subscribe(eventHandler);
        return this;
    }

    public Async.AsyncBody gotoPosition(double weight) {
        return async -> {
            this.setPosition(weight);
            this._isAsync = true;
            this.subscribeBecomeIdleEvent(() -> {
                this._isAsync = false;
                async.finish();
                return true;
            });
        };
    }

    public Async.AsyncBody gotoPosition(double weight, double time) {
        return async -> {
            this.setPosition(weight);
            this._isAsync = true;
            DcMotorWrapper.timeManager.subscribeTimeEvent(time, () -> {
                this._isAsync = false;
                async.finish();
            });
        };
    }

    public Async.AsyncBody gotoPosition(int weight) {
        return async -> {
            this.setPosition(weight);
            this._isAsync = true;
            this.subscribeBecomeIdleEvent(() -> {
                this._isAsync = false;
                async.finish();
                return true;
            });
        };
    }

    public Async.AsyncBody gotoPosition(int weight, double time) {
        return async -> {
            this.setPosition(weight);
            this._isAsync = true;
            DcMotorWrapper.timeManager.subscribeTimeEvent(time, () -> {
                this._isAsync = false;
                async.finish();
            });
        };
    }

    public DcMotor getDcMotor() {
        return this._dcMotor;
    }

    public int getLowerBound() {
        return this._lowerBound;
    }

    public int getUpperBound() {
        return this._upperBound;
    }

    public double getForwardPower() {
        return this._forwardPower;
    }

    public double getReversePower() {
        return this._reversePower;
    }

    public boolean isBusy() {
        return this._isBusy;
    }

    public boolean isAsync() {
        return this._isAsync;
    }

    public int getPosition() {
        return this._position;
    }

    public int getTargetPosition() {
        return this._targetPosition;
    }

    private void updatePosition() {
        this._position = this._dcMotor.getCurrentPosition();
    }

    private void updateIsBusy() {
        boolean nowBusy = this._dcMotor.isBusy();
        if (!this._isBusy && nowBusy) this.becomeBusyEventManager.execute();
        if (this._isBusy && !nowBusy) this.becomeIdleEventManager.execute();
        this._isBusy = nowBusy;
    }

    public void update() {
        this.updatePosition();
        this.updateIsBusy();
    }
}