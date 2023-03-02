package org.firstinspires.ftc.teamcode.command.claw;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;

public class Release extends CommandBase {
    private final ClawSubsystem claw;

    public Release(ClawSubsystem claw){
        this.claw = claw;

        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.release();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
