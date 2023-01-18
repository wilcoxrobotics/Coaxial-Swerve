package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import org.firstinspires.ftc.teamcode.lib.Junctions;
import org.firstinspires.ftc.teamcode.lib.errors.NotAValidJunctionException;
import org.firstinspires.ftc.teamcode.lib.errors.NotAValidTickAmountException;
import org.firstinspires.ftc.teamcode.subsystem.parts.Slides;
import org.firstinspires.ftc.teamcode.subsystem.parts.V4bClaw;

public class ArmSubsystem {
    private MotorEx slideR;
    private MotorEx slideL;
    protected CRServoImplEx v4bMoveRight;
    protected CRServoImplEx v4bMoveLeft;
    protected CRServoImplEx clawPitch;
    protected CRServoImplEx clawOpen;
    protected CRServoImplEx clawRot;

    public V4bClaw claw;
    public Slides slides;
    public ArmSubsystem (MotorEx linSlideLeft, MotorEx linSlideRight, CRServoImplEx v4bMoveRight, CRServoImplEx v4bMoveLeft, CRServoImplEx clawPitch, CRServoImplEx clawOpen, CRServoImplEx clawRot) {
        this.slideL = linSlideLeft;
        this.slideR = linSlideRight;
        this.v4bMoveLeft = v4bMoveLeft;
        this.v4bMoveRight = v4bMoveRight;
        this.clawOpen = clawOpen;
        this.clawPitch = clawPitch;
        this.clawRot = clawRot;

        slides = new Slides(this.slideL, this.slideR);
        claw = new V4bClaw(this.v4bMoveRight, this.v4bMoveLeft, this.clawPitch, this.clawOpen, this.clawRot);
    }

    private void resetAll() {
        //slides reset
        //v4b auto goes to in position and opens claw
    }

    private void goToJunction(Junctions junction) throws NotAValidJunctionException {
        switch (junction) {
            case HIGH:
                //if claw is not closed close claw
                //go to high

                break;
            case MEDIUM:
                //if claw is not closed close claw
                //go to medium
                break;
            case LOW:
                //if claw is not closed close claw
                //go to low
                break;
            case GROUND:
                //if claw is not closed close claw
                //slides go to ground
                break;
            default:
                throw new NotAValidJunctionException();
        }
    }
    private void goTo(int ticks) throws NotAValidTickAmountException {
        if (ticks > 2500) {
            throw new NotAValidTickAmountException();
        }
        if(ticks < -2500) {
            throw new NotAValidTickAmountException();
        }
        //slides go to ticks
    }


}
