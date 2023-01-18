package org.firstinspires.ftc.teamcode.subsystem.parts;

import android.util.Log;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.lib.Junctions;
import org.firstinspires.ftc.teamcode.lib.errors.NotAValidSetPointException;
import org.firstinspires.ftc.teamcode.lib.errors.NotAValidTickAmountException;
import org.firstinspires.ftc.teamcode.lib.onbot.DcMotorWrapper;

public class Slides {
    DcMotorWrapper linSlideLeft, linSlideRight;
    DcMotorEx manSlideRight, manSlideLeft;
    private final double manualSlideSpeed = 0.5; //higher is faster

    MotorEx slideL, slideR;

    //TODO tune height values
    public static int RESET = 0;
    public static int GROUND = 0;
    public static int LOW = 0;
    public static int MEDIUM = 0;
    public static int HIGH = 0;

    public static int SlidePosMax = 0; //need to define max height

    public static int SlidePosMin = 0; //need to define min height

    public static double calc;



    public static double slide_P = 0;

    public static double slide_I = 0;

    public static double slide_D = 0;

    //try using ftc dashboard to get these values
    public static double maxVel = 1000;

    public static double maxAccel = 1000;

    //https://www.ctrlaltftc.com/the-pid-controller/tuning-methods-of-a-pid-controller

    //both motors need to be same spec

    private final ProfiledPIDController slide_pidL = new ProfiledPIDController(slide_P, slide_I, slide_D,
            new TrapezoidProfile.Constraints(maxVel, maxAccel));

    private final ProfiledPIDController slide_pidR = new ProfiledPIDController(slide_P, slide_I, slide_D,
            new TrapezoidProfile.Constraints(maxVel, maxAccel));

    public static double tolerance = 10;

    private double out_left;

    private double out_right;


    public static Junctions currentGoal = Junctions.RESET;

    public Slides(MotorEx slideL, MotorEx slideR) {
        slide_pidL.setTolerance(tolerance);
        slide_pidR.setTolerance(tolerance);

        //todo make sure this works right (on start should just stay at the bottom)
        slide_pidL.setGoal(0);
        slide_pidR.setGoal(0);

    }


    public void moveToJunction(Junctions junction) throws NotAValidTickAmountException {
        currentGoal = junction;
        switch(junction){
            case RESET:
                setSlidesPID(RESET);
                break;
            case GROUND:
                setSlidesPID(GROUND);
                break;
            case LOW:
                setSlidesPID(LOW);
                break;
            case MEDIUM:
                setSlidesPID(MEDIUM);
                break;
            case HIGH:
                setSlidesPID(HIGH);
                break;
        }
        out_left = slide_pidL.calculate(slideL.getCurrentPosition());

        //only works if both motors synced
        while(!slide_pidL.atGoal()){
            out_left = slide_pidL.calculate(slideL.getCurrentPosition());
            Log.d("output", "" + out_left);
            Log.d("error", "" + slide_pidL.getPositionError());
            Log.d("encoder", "" + slideL.getCurrentPosition());

            out_right = out_left;

            slideL.set(out_left);
            slideR.set(out_right);
        }

    }

    public void setSlidesPID (int goal) throws NotAValidTickAmountException {
        if(goal>2500 || goal<-2500){
            throw new NotAValidTickAmountException();
        }
        slide_pidL.setGoal(goal);
        slide_pidR.setGoal(goal);
    }

    public void changeSetPoint(double input) throws NotAValidSetPointException {
        try {
            calc = slideL.getCurrentPosition() + input * manualSlideSpeed;
            // is this logic correct?
            if (calc > SlidePosMax) {
                slide_pidL.setGoal((SlidePosMax));
                slide_pidR.setGoal((SlidePosMax));
                Log.d("setpoint left", "" + slide_pidL.getSetpoint().position);
                Log.d("setpoint right", "" + slide_pidR.getSetpoint().position);
            } else if (calc < SlidePosMin) {
                slide_pidL.setGoal((SlidePosMin));
                slide_pidR.setGoal((SlidePosMin));
                Log.d("setpoint left", "" + slide_pidL.getSetpoint().position);
                Log.d("setpoint right", "" + slide_pidR.getSetpoint().position);
            } else {
                slide_pidL.setGoal((int) (slideL.getCurrentPosition() + input * manualSlideSpeed));
                slide_pidR.setGoal((int) (slideR.getCurrentPosition() + input * manualSlideSpeed));
                Log.d("setpoint left", "" + slide_pidL.getSetpoint().position);
                Log.d("setpoint right", "" + slide_pidR.getSetpoint().position);
            }
        } catch(Exception e) {
            throw new NotAValidSetPointException();
        }
    }



    public int getSlideLEncoder(){
        return slideL.getCurrentPosition();
    }
    public int getSlideREncoder(){
        return slideR.getCurrentPosition();
    }

    public double getSlideLPower(){
        return out_left;
    }
    public double getSlideRPower(){
        return out_right;
    }

    public double getSlideLError(){return slide_pidL.getPositionError();}

    public double getSlideRError(){return slide_pidR.getPositionError();}

}
