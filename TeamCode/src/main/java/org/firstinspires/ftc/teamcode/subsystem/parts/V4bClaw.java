package org.firstinspires.ftc.teamcode.subsystem.parts;

import com.qualcomm.robotcore.hardware.CRServoImplEx;

public class V4bClaw {
    protected CRServoImplEx v4bMoveRight;
    public static double v4bMoveRightStart = -1, v4bMoveRightEnd = 0.2;

    protected CRServoImplEx v4bMoveLeft;
    public static double v4bMoveLeftStart = -1, v4bMoveLeftEnd = 0.2;

    protected CRServoImplEx clawPitch;
    public static double clawPitchStart = -1, clawPitchEnd = 0.5;

    protected CRServoImplEx clawOpen;
    public static double clawOpenStart = 0, clawOpenEnd = 1;

    protected CRServoImplEx clawRot;
    public static double clawRotStart = -.7, clawRotEnd = 1;
    public V4bClaw(CRServoImplEx v4bMoveRight, CRServoImplEx v4bMoveLeft, CRServoImplEx clawPitch, CRServoImplEx clawOpen, CRServoImplEx clawRot) {
        this.v4bMoveRight = v4bMoveRight;
        this.v4bMoveLeft = v4bMoveLeft;
        this.clawPitch = clawPitch;
        this.clawOpen = clawOpen;
        this.clawRot = clawRot;
    }

    /* Example Code for this implementation
    boolean rotButtonPrevState = gamepad1.right_bumper, clawButtonPrevState = gamepad1.left_bumper;
    boolean rotButtonState = true, clawButtonState = false;
     */
    private void inToOut(boolean clawButtonState, boolean rotButtonState) {
        if(clawButtonState){
            clawOpen.setPower(clawButtonState ? clawOpenStart : clawOpenEnd);
        }
        clawRot.setPower(rotButtonState ? clawRotStart : clawRotEnd);
        clawPitch.setPower(rotButtonState ? clawPitchStart : clawPitchEnd);
        v4bMoveLeft.setPower(!rotButtonState ? v4bMoveLeftStart : v4bMoveLeftEnd);
        v4bMoveRight.setPower(!rotButtonState ? v4bMoveRightStart : v4bMoveRightEnd);
    }

    private void activateClaw(boolean clawButtonState){
        clawOpen.setPower(clawButtonState ? clawOpenStart : clawOpenEnd);
    }
}
