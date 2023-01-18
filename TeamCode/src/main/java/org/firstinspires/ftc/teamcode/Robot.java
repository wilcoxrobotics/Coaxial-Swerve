package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class Robot {
    DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, v4bright, v4bleft;

    protected CRServoImplEx v4bMoveRight;
    protected CRServoImplEx v4bMoveLeft;
    protected CRServoImplEx clawPitch;
    protected CRServoImplEx clawOpen;
    protected CRServoImplEx clawRot;
    ArmSubsystem arm;
  public Robot(HardwareMap hardwareMap) {
      motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
      motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftRear");
      motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
      motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get ("rightRear");

      v4bright = (DcMotorEx) hardwareMap.dcMotor.get( "rightSlide");
      v4bleft = (DcMotorEx) hardwareMap.dcMotor.get("leftSlide");

      v4bMoveLeft = (CRServoImplEx) hardwareMap.crservo.get("leftArm");
      v4bMoveRight = (CRServoImplEx) hardwareMap.crservo.get("rightArm");
      clawRot = (CRServoImplEx) hardwareMap.crservo.get("turret");
      clawPitch = (CRServoImplEx) hardwareMap.crservo.get("pitch");
      clawOpen = (CRServoImplEx) hardwareMap.crservo.get("claw");

      arm = new ArmSubsystem(v4bleft, v4bright, v4bMoveRight, v4bMoveLeft, clawPitch, clawOpen, clawRot);

  }
}
