package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.rpm.initializeFlywheels;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name="TestTeleOp")
public class TestTeleOp extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftFlyWheel = null;
    private DcMotor rightFlyWheel = null;
    private DcMotor intake = null;
    private Servo pitchLeft = null;
    private Servo pitchRight = null;
    private CRServo beltFront = null;//CRServo means continuous rotation for all you new gens
    private CRServo beltBack = null;
    private Servo timingServo = null;
    private Servo doorServo = null;


    @Override
    public void runOpMode() {
        //base
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servos
        pitchLeft = hardwareMap.get(Servo.class, "pitchLeft");
        pitchRight = hardwareMap.get(Servo.class, "pitchRight");
        timingServo = hardwareMap.get(Servo.class, "pitchRight");
        doorServo = hardwareMap.get(Servo.class, "pitchRight");
        beltFront = hardwareMap.get(CRServo.class, "beltFront");
        beltBack = hardwareMap.get(CRServo.class, "beltBack");

        //Fly-wheel
        leftFlyWheel = hardwareMap.get(DcMotor.class, "leftFlyWheel");
        leftFlyWheel.setDirection(DcMotor.Direction.FORWARD);
        leftFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFlyWheel = hardwareMap.get(DcMotor.class, "rightFlyWheel");
        rightFlyWheel.setDirection(DcMotor.Direction.REVERSE);
        leftFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Intakes
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {

            double currentTime = getRuntime();
            // --------------------------- WHEELS --------------------------- //
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = Math.pow(-gamepad1.left_stick_y, 3);  // Note: pushing stick forward gives negative value
            double lateral = Math.pow(gamepad1.left_stick_x, 3);
            double yaw = Math.pow(gamepad1.right_stick_x, 3);
            double leftFrontPower = gamepad1.dpad_left ? 1 : axial + lateral + yaw;
            double rightFrontPower = gamepad1.dpad_right ? 1 : axial - lateral - yaw;
            double leftBackPower = gamepad1.dpad_down ? 1 : axial - lateral + yaw;
            double rightBackPower = gamepad1.dpad_up ? 1 : axial + lateral - yaw;

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Reduced Power Mode
            if (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0) {
                leftFrontPower /= 2;
                rightFrontPower /= 2;
                leftBackPower /= 2;
                rightBackPower /= 2;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            //intake for player1 does both belt and intake motor
            //forward
            if(gamepad2.right_trigger>0){
                intake.setPower(1);
                beltFront.setPower(gamepad2.right_trigger);
                beltBack.setPower(gamepad2.right_trigger);
            }
            //reverse
            if(gamepad2.left_trigger>0){
                intake.setPower(-1);
                beltFront.setPower(-gamepad2.right_trigger);
                beltBack.setPower(-gamepad2.right_trigger);
            }
            //Control for the conveyor belt for player 2
            if(gamepad2.right_stick_y>0 || gamepad2.right_stick_y<0){
                beltFront.setPower(-gamepad2.right_stick_y);
                beltBack.setPower(-gamepad2.right_stick_y);
            }

            //control for only intake for player 2
            if(gamepad2.left_stick_y>0 || gamepad2.left_stick_y<0){
                beltFront.setPower(-gamepad2.left_stick_y);
                beltBack.setPower(-gamepad2.left_stick_y);
            }

            initializeFlywheels();

            // --------------------------- TELEMETRY --------------------------- //

            telemetry.addData("Front left/Right", "%4.2f, %4.2f",
                    leftFrontDrive.getPower(), rightFrontDrive.getPower());
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f",
                    leftBackDrive.getPower(), rightBackDrive.getPower());

            telemetry.addData("Front/Back Belt", "%4.2f, %4.2f",
                    beltFront.getPower(), beltBack.getPower());
            telemetry.addData("Intake", "%4.2f", intake.getPower());

            telemetry.addData("Flywheel RPM Left/Right", "%4.2f,%4.2f",rpm.getLeftRPM(),rpm.getRightRPM());

            telemetry.update();
        }
    }

    // Dedicated method for the PID loop
    private void leftFlyWheelPIDLoop(double targetRPM) {
        pid left = new pid(leftFlyWheel);
        while (opModeIsActive()) {
            //works when robot is not moving
            if (gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0) {
                double power = left.update(targetRPM);
                leftFlyWheel.setPower(power);
            }
            sleep(1);
        }
    }
    private void rightFlyWheelPIDLoop(double targetRPM) {
        pid right = new pid(rightFlyWheel);
        while (opModeIsActive()) {
            //works when robot is not moving
            if (gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0) {
                double power = right.update(targetRPM);
                leftFlyWheel.setPower(power);
            }
            sleep(1);
        }
    }
    private void setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }
}
