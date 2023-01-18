package org.firstinspires.ftc.teamcode.lib.errors;

import com.acmerobotics.dashboard.message.Message;
import com.acmerobotics.dashboard.message.MessageType;

public class NotAValidJunctionException extends Exception {
    public NotAValidJunctionException() {
        super("This is not a valid Junction!");
    }
}
