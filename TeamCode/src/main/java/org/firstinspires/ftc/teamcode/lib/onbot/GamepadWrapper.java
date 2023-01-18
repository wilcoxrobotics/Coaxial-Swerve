package org.firstinspires.ftc.teamcode.lib.onbot;

import com.qualcomm.robotcore.hardware.Gamepad;



public class GamepadWrapper {
    private Gamepad _gamepad;
    private boolean _isAPressed = false;
    private boolean _isBPressed = false;
    private boolean _isXPressed = false;
    private boolean _isYPressed = false;
    private boolean _isDPressed = false;
    private boolean _isLPressed = false;
    private boolean _isRPressed = false;
    private boolean _isUPressed = false;
    private boolean _isLbPressed = false;
    private boolean _isRbPressed = false;
    private EventManager aPressedEventManager;
    private EventManager bPressedEventManager;
    private EventManager xPressedEventManager;
    private EventManager yPressedEventManager;
    private EventManager aReleasedEventManager;
    private EventManager bReleasedEventManager;
    private EventManager xReleasedEventManager;
    private EventManager yReleasedEventManager;
    private EventManager dPressedEventManager;
    private EventManager lPressedEventManager;
    private EventManager rPressedEventManager;
    private EventManager uPressedEventManager;
    private EventManager dReleasedEventManager;
    private EventManager lReleasedEventManager;
    private EventManager rReleasedEventManager;
    private EventManager uReleasedEventManager;
    private EventManager lbPressedEventManager;
    private EventManager rbPressedEventManager;
    private EventManager lbReleasedEventManager;
    private EventManager rbReleasedEventManager;

    public GamepadWrapper() {
        this.aPressedEventManager = new EventManager();
        this.bPressedEventManager = new EventManager();
        this.xPressedEventManager = new EventManager();
        this.yPressedEventManager = new EventManager();
        this.aReleasedEventManager = new EventManager();
        this.bReleasedEventManager = new EventManager();
        this.xReleasedEventManager = new EventManager();
        this.yReleasedEventManager = new EventManager();
        this.dPressedEventManager = new EventManager();
        this.lPressedEventManager = new EventManager();
        this.rPressedEventManager = new EventManager();
        this.uPressedEventManager = new EventManager();
        this.dReleasedEventManager = new EventManager();
        this.lReleasedEventManager = new EventManager();
        this.rReleasedEventManager = new EventManager();
        this.uReleasedEventManager = new EventManager();
        this.lbPressedEventManager = new EventManager();
        this.rbPressedEventManager = new EventManager();
        this.lbReleasedEventManager = new EventManager();
        this.rbReleasedEventManager = new EventManager();
    }

    public GamepadWrapper setGamepad(Gamepad gamepad) {
        this._gamepad = gamepad;
        return this;
    }

    public GamepadWrapper subscribeAPressedEvent(EventManager.EventHandler eventHandler) {
        this.aPressedEventManager.subscribe(eventHandler);
        return this;
    }

    public GamepadWrapper subscribeBPressedEvent(EventManager.EventHandler eventHandler) {
        this.bPressedEventManager.subscribe(eventHandler);
        return this;
    }

    public GamepadWrapper subscribeXPressedEvent(EventManager.EventHandler eventHandler) {
        this.xPressedEventManager.subscribe(eventHandler);
        return this;
    }

    public GamepadWrapper subscribeYPressedEvent(EventManager.EventHandler eventHandler) {
        this.yPressedEventManager.subscribe(eventHandler);
        return this;
    }

    public GamepadWrapper subscribeAReleasedEvent(EventManager.EventHandler eventHandler) {
        this.aReleasedEventManager.subscribe(eventHandler);
        return this;
    }

    public GamepadWrapper subscribeBReleasedEvent(EventManager.EventHandler eventHandler) {
        this.bReleasedEventManager.subscribe(eventHandler);
        return this;
    }

    public GamepadWrapper subscribeXReleasedEvent(EventManager.EventHandler eventHandler) {
        this.xReleasedEventManager.subscribe(eventHandler);
        return this;
    }

    public GamepadWrapper subscribeYReleasedEvent(EventManager.EventHandler eventHandler) {
        this.yReleasedEventManager.subscribe(eventHandler);
        return this;
    }

    public GamepadWrapper subscribeDPressedEvent(EventManager.EventHandler eventHandler) {
        this.dPressedEventManager.subscribe(eventHandler);
        return this;
    }

    public GamepadWrapper subscribeLPressedEvent(EventManager.EventHandler eventHandler) {
        this.lPressedEventManager.subscribe(eventHandler);
        return this;
    }

    public GamepadWrapper subscribeRPressedEvent(EventManager.EventHandler eventHandler) {
        this.rPressedEventManager.subscribe(eventHandler);
        return this;
    }

    public GamepadWrapper subscribeUPressedEvent(EventManager.EventHandler eventHandler) {
        this.uPressedEventManager.subscribe(eventHandler);
        return this;
    }

    public GamepadWrapper subscribeDReleasedEvent(EventManager.EventHandler eventHandler) {
        this.dReleasedEventManager.subscribe(eventHandler);
        return this;
    }

    public GamepadWrapper subscribeLReleasedEvent(EventManager.EventHandler eventHandler) {
        this.lReleasedEventManager.subscribe(eventHandler);
        return this;
    }

    public GamepadWrapper subscribeRReleasedEvent(EventManager.EventHandler eventHandler) {
        this.rReleasedEventManager.subscribe(eventHandler);
        return this;
    }

    public GamepadWrapper subscribeUReleasedEvent(EventManager.EventHandler eventHandler) {
        this.uReleasedEventManager.subscribe(eventHandler);
        return this;
    }

    public GamepadWrapper subscribeLbPressedEvent(EventManager.EventHandler eventHandler) {
        this.lbPressedEventManager.subscribe(eventHandler);
        return this;
    }

    public GamepadWrapper subscribeRbPressedEvent(EventManager.EventHandler eventHandler) {
        this.rbPressedEventManager.subscribe(eventHandler);
        return this;
    }

    public GamepadWrapper subscribeLbReleasedEvent(EventManager.EventHandler eventHandler) {
        this.lbReleasedEventManager.subscribe(eventHandler);
        return this;
    }

    public GamepadWrapper subscribeRbReleasedEvent(EventManager.EventHandler eventHandler) {
        this.rbReleasedEventManager.subscribe(eventHandler);
        return this;
    }

    public Gamepad getGamepad() {
        return this._gamepad;
    }

    public boolean isAPressed() {
        return this._isAPressed;
    }

    public boolean isBPressed() {
        return this._isBPressed;
    }

    public boolean isXPressed() {
        return this._isXPressed;
    }

    public boolean isYPressed() {
        return this._isYPressed;
    }

    public boolean isDPressed() {
        return this._isDPressed;
    }

    public boolean isLPressed() {
        return this._isLPressed;
    }

    public boolean isRPressed() {
        return this._isRPressed;
    }

    public boolean isUPressed() {
        return this._isUPressed;
    }

    public boolean isLbPressed() {
        return this._isLbPressed;
    }

    public boolean isRbPressed() {
        return this._isRbPressed;
    }

    private void updateLeftButtons() {
        boolean nowDPressed = this._gamepad.dpad_down;
        boolean nowLPressed = this._gamepad.dpad_left;
        boolean nowRPressed = this._gamepad.dpad_right;
        boolean nowUPressed = this._gamepad.dpad_up;
        if (!this._isDPressed && nowDPressed) this.dPressedEventManager.execute();
        if (!this._isLPressed && nowLPressed) this.lPressedEventManager.execute();
        if (!this._isRPressed && nowRPressed) this.rPressedEventManager.execute();
        if (!this._isUPressed && nowUPressed) this.uPressedEventManager.execute();
        if (this._isDPressed && !nowDPressed) this.dReleasedEventManager.execute();
        if (this._isLPressed && !nowLPressed) this.lReleasedEventManager.execute();
        if (this._isRPressed && !nowRPressed) this.rReleasedEventManager.execute();
        if (this._isUPressed && !nowUPressed) this.uReleasedEventManager.execute();
        this._isDPressed = nowDPressed;
        this._isLPressed = nowLPressed;
        this._isRPressed = nowRPressed;
        this._isUPressed = nowUPressed;
    }

    private void updateRightButtons() {
        boolean nowAPressed = this._gamepad.a;
        boolean nowBPressed = this._gamepad.b;
        boolean nowXPressed = this._gamepad.x;
        boolean nowYPressed = this._gamepad.y;
        if (!this._isAPressed && nowAPressed) this.aPressedEventManager.execute();
        if (!this._isBPressed && nowBPressed) this.bPressedEventManager.execute();
        if (!this._isXPressed && nowXPressed) this.xPressedEventManager.execute();
        if (!this._isYPressed && nowYPressed) this.yPressedEventManager.execute();
        if (this._isAPressed && !nowAPressed) this.aReleasedEventManager.execute();
        if (this._isBPressed && !nowBPressed) this.bReleasedEventManager.execute();
        if (this._isXPressed && !nowXPressed) this.xReleasedEventManager.execute();
        if (this._isYPressed && !nowYPressed) this.yReleasedEventManager.execute();
        this._isAPressed = nowAPressed;
        this._isBPressed = nowBPressed;
        this._isXPressed = nowXPressed;
        this._isYPressed = nowYPressed;
    }

    private void updateBumpers() {
        boolean nowLbPressed = this._gamepad.left_bumper;
        boolean nowRbPressed = this._gamepad.right_bumper;
        if (!this._isLbPressed && nowLbPressed) this.lbPressedEventManager.execute();
        if (!this._isRbPressed && nowRbPressed) this.rbPressedEventManager.execute();
        if (this._isLbPressed && !nowLbPressed) this.lbReleasedEventManager.execute();
        if (this._isRbPressed && !nowRbPressed) this.rbReleasedEventManager.execute();
        this._isLbPressed = nowLbPressed;
        this._isRbPressed = nowRbPressed;
    }

    public void update() {
        this.updateLeftButtons();
        this.updateRightButtons();
        this.updateBumpers();
    }
}