package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.drive.DriveRobotCentric;

@TeleOp(name = "Drive OpMode", group = "TeleOp")
public class DriveOpMode extends DriveBaseOpMode{
    private DriveRobotCentric robotCentricDrive;
    @Override
    public void initialize() {
        super.initialize();
        robotCentricDrive = new DriveRobotCentric(drive, gamepadEx1::getLeftX,
                gamepadEx1::getRightX, gamepadEx1::getLeftY);


        register(drive);
        drive.setDefaultCommand(robotCentricDrive);
    }
}
