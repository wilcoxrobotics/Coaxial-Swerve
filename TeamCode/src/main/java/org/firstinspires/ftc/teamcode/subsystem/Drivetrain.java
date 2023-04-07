package org.firstinspires.ftc.teamcode.subsystem;


import org.firstinspires.ftc.teamcode.util.geometry.Pose;

public interface Drivetrain {

    void set(Pose pose);

    void set(Pose pose, double maxPower);
}