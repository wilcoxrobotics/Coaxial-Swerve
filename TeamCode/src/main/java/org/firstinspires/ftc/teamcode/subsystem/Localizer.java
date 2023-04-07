package org.firstinspires.ftc.teamcode.subsystem;

import org.firstinspires.ftc.teamcode.util.geometry.Pose;

public interface Localizer {

    void periodic();

    Pose getPos();

    void setPos(Pose pose);
}