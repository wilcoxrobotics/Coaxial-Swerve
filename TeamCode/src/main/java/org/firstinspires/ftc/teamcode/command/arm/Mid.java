package org.firstinspires.ftc.teamcode.command.arm;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class Mid extends CommandBase {
    private final ArmSubsystem arm;

    public Mid(ArmSubsystem arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.mid();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
