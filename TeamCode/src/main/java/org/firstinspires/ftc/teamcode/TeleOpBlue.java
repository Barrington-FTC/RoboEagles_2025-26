package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
@Config
@TeleOp(name="TeleOpBlue")
public class TeleOpBlue extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intake = null;
    private DcMotorEx flyWheel = null;
    private DcMotor turret = null;
    private Servo pitch = null;
    private Limelight3A limeLight = null;
    double ta;
    double tx;
    double vf;
    public static int MAX_TURRET = 2000;
    public static int MIN_TURRET = -2000;
    private PIDControllerRyan turretPID = null;

    @Override
    public void runOpMode(){
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        turret = hardwareMap.get(DcMotor.class, "intake");
        pitch = hardwareMap.get(Servo.class, "pitch");
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        pitch.setDirection(Servo.Direction.FORWARD);
        //needed for pid
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //lime light
        limeLight = hardwareMap.get(Limelight3A.class, "limeLight");
        limeLight.setPollRateHz(50); // This sets how often we ask Limelight for data (100 times per second)
        limeLight.start();
        //0 for blue tracking 1 for red tracking and 2 for finding sorting order.
        limeLight.pipelineSwitch(0);

        setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Thread limeLightThread = new Thread(this::limeLightLoop);
        //Thread aimThread = new Thread(this::aim);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        limeLightThread.start();
        //aimThread.start();
        pitch.setPosition(0);
        while (opModeIsActive()) { // Loop
            // --------------------------- WHEELS --------------------------- //
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = Math.pow(-gamepad1.left_stick_y, 3);  // Note: pushing stick forward gives negative value
            double lateral = Math.pow(gamepad1.left_stick_x, 3);
            double yaw = Math.pow(gamepad1.right_stick_x, 3);
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            if (gamepad1.right_bumper || gamepad1.left_bumper) {
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

            turret.setPower(1);

            if (gamepad1.right_trigger> 0.01){
                intake.setPower(gamepad1.right_trigger);
            }
            else if (gamepad1.left_trigger> 0.01) {
                intake.setPower(-gamepad1.left_trigger);
            }
            else{
                intake.setPower(0);            }

            //-------------------------- TELEMETRY --------------------------- //
            // Show the elapsed game time and wheel power.

            telemetry.addData("Front left/Right", "%4.2f, %4.2f",
                    leftFrontDrive.getPower(), rightFrontDrive.getPower());
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f",
                    leftBackDrive.getPower(), rightBackDrive.getPower());
            telemetry.addData("Intake", "%4.2f",
                    intake.getPower());
            telemetry.addData("ll status", limeLight.isConnected());
            telemetry.addData("ta",ta);
            telemetry.addData("Flywheel", vf);
            telemetry.addData("Pitch", pitch.getPosition());
            telemetry.addData("tx",tx);
            telemetry.addData("Turret", turret.getCurrentPosition());
            telemetry.addData("ll connection",limeLight.getConnectionInfo());
            telemetry.update();
        }
    }
    private void limeLightLoop() {
        while (opModeIsActive()) {
            LLResult result = limeLight.getLatestResult();
            tx = result.getTx();
            ta = result.getTa();
            vf = -76.61854*ta+2245.22264;
            if(ta > 0.1 && ta<5){
                flyWheel.setVelocity(vf);
            }
            sleep(20);
        }
    }
    private void aim(){
        while(opModeIsActive()) {
            int roundedInt = (int) Math.round(tx);
            double power = turretPID.update(0, roundedInt); // Use the D-pad updated TargetPosition
                turret.setPower(power);
            sleep(50);
        }
    }
    // Dedicated method for the PID loop
    private void setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }
}
