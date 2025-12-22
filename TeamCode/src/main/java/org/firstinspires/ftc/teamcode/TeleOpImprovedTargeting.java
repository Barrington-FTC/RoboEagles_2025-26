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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
@TeleOp(name="TeleOpImprovedTargeting")
public class TeleOpImprovedTargeting extends LinearOpMode {
    private DcMotor lfd = null;
    private DcMotor lbd = null;
    private DcMotor rfd = null;
    private DcMotor rbd = null;
    private DcMotor intake = null;
    private DcMotorEx flyWheel = null;
    private DcMotor turret = null;
    private Servo pitch = null;
    private Limelight3A limeLight = null;
    private GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    //for odomitry related function look at documentation:
    //https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf?srsltid=AfmBOooVrKs1eVSKj9v9hnZDigEWX6OUK8exzJzJAaHLzT9edc7YaVKa
    double oldTime = 0;
    double ta;
    double tx;
    double vf;

    double totalTargetDistance; //net distance from center of robot to red or blue target

    double x;//x value of odometry
    double y;//y value of odometry
    double h;//heading value of odometry
    double xVel;//x velocity of odometry
    double yVel;//y velocity of odometry
    double xStarting;//based on ending position of auto
    double yStarting;//based on ending position of auto
    double hStarting;//based on ending position of auto

    double redX;// aiming point x value
    double redY;// aiming point y value
    double blueX;// aiming point x value
    double blueY;// aiming point y value
    public static int MAX_TURRET = 2000;
    public static int MIN_TURRET = -2000;
    private PIDControllerRyan turretPID = null;

    @Override
    public void runOpMode(){
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setOffsets(0,0);//look at cad for exact offsets need to be as accurate as possible.
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        lfd = hardwareMap.get(DcMotor.class, "lfd");
        lbd = hardwareMap.get(DcMotor.class, "lbd");
        rfd = hardwareMap.get(DcMotor.class, "rfd");
        rbd = hardwareMap.get(DcMotor.class, "rbd");
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        turret = hardwareMap.get(DcMotor.class, "intake");
        pitch = hardwareMap.get(Servo.class, "pitch");
        lfd.setDirection(DcMotor.Direction.FORWARD);
        lbd.setDirection(DcMotor.Direction.FORWARD);
        rfd.setDirection(DcMotor.Direction.REVERSE);
        rbd.setDirection(DcMotor.Direction.FORWARD);
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


        odo.resetPosAndIMU();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        pitch.setPosition(0);
        while (opModeIsActive()) { // Loop
            //odo
            odo.update();
            Pose2D pos = odo.getPosition();
            x = pos.getX(DistanceUnit.METER) + xStarting;
            y = pos.getY(DistanceUnit.METER) + yStarting;
            h = pos.getHeading(AngleUnit.RADIANS) + hStarting;
            //Targeting using odo
            //for red side
            totalTargetDistance = Math.sqrt((Math.pow((x-redX),2) + Math.pow((y-redY),2)));
            //for blue
            totalTargetDistance = Math.sqrt((Math.pow((x-blueX),2) + Math.pow((y-blueY),2)));



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
            lfd.setPower(leftFrontPower);
            rfd.setPower(rightFrontPower);
            lbd.setPower(leftBackPower);
            rbd.setPower(rightBackPower);

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
                    lfd.getPower(), rfd.getPower());
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f",
                    lbd.getPower(), rbd.getPower());
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
    // Dedicated method for the PID loop
    private void setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        lfd.setZeroPowerBehavior(behavior);
        lbd.setZeroPowerBehavior(behavior);
        rfd.setZeroPowerBehavior(behavior);
        rbd.setZeroPowerBehavior(behavior);
    }
}
