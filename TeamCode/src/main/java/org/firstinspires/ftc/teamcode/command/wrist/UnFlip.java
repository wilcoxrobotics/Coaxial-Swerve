package org.firstinspires.ftc.teamcode.command.wrist;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;

public class UnFlip extends CommandBase {
    private final WristSubsystem wrist;

    public UnFlip(WristSubsystem wrist){
        this.wrist = wrist;
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.home();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
