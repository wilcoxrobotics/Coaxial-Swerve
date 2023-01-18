package org.firstinspires.ftc.teamcode.lib.errors;

public class NotAValidTickAmountException extends Exception{
    public NotAValidTickAmountException() {
        super("No Valid Tick Amount provided!");
    }
}
