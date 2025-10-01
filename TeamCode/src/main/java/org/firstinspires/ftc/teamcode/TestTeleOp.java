package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.rpm.initializeFlywheels;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;
import java.lang.Math;
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
    //Pid
    private int targetRPM = 0;
    private pid leftPid = null;
    private pid rightPid = null;
    //Lime Light
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private Limelight3A limelight;
    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;
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
        //PID
        leftPid = new pid(0,0,0,0,leftFlyWheel);
        rightPid = new pid(0,0,0,0,rightFlyWheel);
        Thread LeftPIDThread = new Thread(this::leftFlyWheelPIDLoop);
        Thread RightPIDThread = new Thread(this::rightFlyWheelPIDLoop);
        //April Tag
        initAprilTag();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(1);// look here for the recommended settings.
        waitForStart();



        while (opModeIsActive()) {
            telemetryAprilTag();
            telemetry.update();
            initializeFlywheels();
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
            //Lime Light
            LLResult result = limelight.getLatestResult();
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            if (result != null) {
                // Access general information
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
                int id;
                double xa;// x angle right or left
                double ya;//y angle up or down should always be up though
                double distance;
                if (result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        id = fiducial.getFiducialId(); // The ID number of the fiducial
                        xa = fiducial.getTargetXDegrees(); // Where it is (left-right)
                        xa= Math.toRadians(xa);
                        ya = fiducial.getTargetYDegrees(); // Where it is (up-down)
                        ya= Math.toRadians(ya);

                        distance =
                        telemetry.addData("Fiducial " + id, "is " + distance + " meters away");
                    }
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("tync", result.getTyNC());


                    // Access fiducial results. dont know if we need this
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }

                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();
        }
        limelight.stop();
            if(gamepad2.a){
                // add limlight calculations here
                targetRPM = 3000;

            }
            else{
                visionPortal.stopStreaming();
            }

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
    //April Tag example from website
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.FRONT);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);

    }
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }// end of Apirl tag

    //PID
    private void leftFlyWheelPIDLoop() {
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

    private void rightFlyWheelPIDLoop() {
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
    // End of PID
    private void setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }
}

