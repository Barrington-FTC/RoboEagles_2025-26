package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class rpm {

    private static String type = null;


    private static DcMotor leftFlyWheel = null;
    private static DcMotor rightFlyWheel = null;

    private static ElapsedTime timer = new ElapsedTime();
    private static double leftFlywheelPrevTime = 0;
    private static int leftFlywheelPrevEncoder = 0;
    private static double rightFlywheelPrevTime = 0;
    private static int rightFlywheelPrevEncoder = 0;

    public static final double FLYWHEEL_MOTOR_TICKS_PER_REV = 28;
    public rpm(String type) {
        this.type = type;

    }
    public static void initializeFlywheels() {
        leftFlyWheel.setDirection(DcMotor.Direction.FORWARD); // Or REVERSE
        rightFlyWheel.setDirection(DcMotor.Direction.REVERSE); // Or FORWARD

        if (leftFlyWheel != null) {
            leftFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFlywheelPrevEncoder = leftFlyWheel.getCurrentPosition();
        }
        if (rightFlyWheel != null) {
            rightFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFlywheelPrevEncoder = rightFlyWheel.getCurrentPosition();
        }

        timer.reset();
        leftFlywheelPrevTime = timer.seconds();
        rightFlywheelPrevTime = timer.seconds();
    }


    public static String getType() {
        return type;
    }

    public static double getLeftRPM() {
        if (leftFlyWheel == null) {
            return 0;
        }

        double currentTime = timer.seconds();
        int currentLeftEncoder = leftFlyWheel.getCurrentPosition();

        double leftDt = currentTime - leftFlywheelPrevTime;
        double leftFlywheelRPM = 0;

        if (leftDt > 0.001) { // Avoid division by zero or tiny dt, check for a minimal time interval
            double deltaTicksLeft = currentLeftEncoder - leftFlywheelPrevEncoder;
            leftFlywheelRPM = (deltaTicksLeft / FLYWHEEL_MOTOR_TICKS_PER_REV) / (leftDt / 60.0);
        }

        leftFlywheelPrevTime = currentTime;
        leftFlywheelPrevEncoder = currentLeftEncoder;

        return leftFlywheelRPM;
    }

    public static double getRightRPM() {
        if (rightFlyWheel == null) {
            return 0;
        }

        double currentTime = timer.seconds();
        int currentRightEncoder = rightFlyWheel.getCurrentPosition();

        double rightDt = currentTime - rightFlywheelPrevTime;
        double rightFlywheelRPM = 0;

        if (rightDt > 0.001) {
            double deltaTicksRight = currentRightEncoder - rightFlywheelPrevEncoder;
            rightFlywheelRPM = (deltaTicksRight / FLYWHEEL_MOTOR_TICKS_PER_REV) / (rightDt / 60.0);
        }

        rightFlywheelPrevTime = currentTime;
        rightFlywheelPrevEncoder = currentRightEncoder;

        return rightFlywheelRPM;
    }

    public static void setLeftFlywheelPower(double power) {
        if (leftFlyWheel != null) {
            leftFlyWheel.setPower(power);
        }
    }

    public static void setRightFlywheelPower(double power) {
        if (rightFlyWheel != null) {
            rightFlyWheel.setPower(power);
        }
    }

    public static void setFlywheelPower(double power) {
        setLeftFlywheelPower(power);
        setRightFlywheelPower(power);
    }
}
