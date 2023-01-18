package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import org.firstinspires.ftc.teamcode.lib.onbot.DcMotorWrapper;

public class ArmSubsystem {
    private DcMotorWrapper linSlideLeft;
    private DcMotorWrapper linSlideRight;
    protected CRServoImplEx v4bMoveRight;
    protected CRServoImplEx v4bMoveLeft;
    protected CRServoImplEx clawPitch;
    protected CRServoImplEx clawOpen;
    protected CRServoImplEx clawRot;
    public ArmSubsystem (DcMotorWrapper linSlideLeft,DcMotorWrapper linSlideRight,CRServoImplEx v4bMoveRight, CRServoImplEx v4bMoveLeft, CRServoImplEx clawPitch, CRServoImplEx clawOpen, CRServoImplEx clawRot) {
        this.v4bMoveLeft = v4bMoveLeft;
        this.v4bMoveRight = v4bMoveRight;
        this.clawOpen = clawOpen;
        this.clawPitch = clawPitch;
        this.clawRot = clawRot;
    }
}
