package org.firstinspires.ftc.teamcode.drive.Autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.Globals;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.Location;
import org.firstinspires.ftc.teamcode.vision.PropPipeLine;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
@Config
public class BlueFront extends LinearOpMode {
    static Servo Arm;
    public static Servo LeftGrip;
    public static Servo RightGrip;
    public static DcMotorEx Lift;
    public static DcMotorEx Rotate;
    static Servo Drone;

    // ============================= Arm Intake and Drop Value =============================
    public static double armIntake = 0.27;

    // ================================= LeftGrip/RightGrip Values ===========================

    public static double LeftGripIntake = 0.6, LeftGripOuttake = 1, RightGripIntake = 0.4, RightGripOuttake = 0;

    //    ========================================= New Implementation ======================================
    private PropPipeLine propPipeLine;
    private VisionPortal portal;
    private Location randomization;


    @Override
    public void runOpMode() throws InterruptedException {

        // ============================ Auto Init ========================================
        SampleTankDrive drive = new SampleTankDrive(hardwareMap); // Using Sample Tank Drive to drive the robot

        // Servo Mechanism
        Arm = hardwareMap.get(Servo.class, "arm");
        Drone = hardwareMap.get(Servo.class, "drone");
        LeftGrip = hardwareMap.get(Servo.class, "LeftGrip");
        RightGrip = hardwareMap.get(Servo.class, "RightGrip");


        // DC Motors Mechanism
        Lift = hardwareMap.get(DcMotorEx.class, "lift");
        Rotate = hardwareMap.get(DcMotorEx.class, "rotate");

//         Using ZeroPowerBehaviour because motor needs to stay at a particular pos
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Resetting the Encoder while Starting
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // ========================= New OpenCv Implementation ============================
//        Globals.IS_AUTO = true;
        Globals.ALLIANCE = Location.BLUE;
        Globals.SIDE = Location.CLOSE;

        propPipeLine = new PropPipeLine();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(propPipeLine)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        while (propPipeLine.getCameraState() != VisionPortal.CameraState.STREAMING && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("initializing... please wait");
            telemetry.update();
        }


        // ============================= While OpMode is in Init Mode ==========================
        if (opModeInInit()) {
            //================= Both Gripper Close

            LeftGrip.setPosition(LeftGripOuttake);
            RightGrip.setPosition(RightGripOuttake);
            Drone.setPosition(0.7);

        }

        while (opModeInInit()) {
            telemetry.addLine("ready");
            telemetry.addData("position", propPipeLine.getLocation());
            telemetry.update();
        }
        randomization = propPipeLine.getLocation();
        portal.close();




        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        drive.setPoseEstimate(startPose);


        TrajectorySequence positionLeft = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> Arm.setPosition(armIntake))
                .waitSeconds(1)
                .forward(9.5) // 10
                .turn(Math.toRadians(21)) // 35
                .addTemporalMarker(() -> RightGrip.setPosition(RightGripIntake)) // Left Grip Open
                .waitSeconds(2)
                .back(5)
                .waitSeconds(10)
                .build();

        TrajectorySequence positionRight = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> Arm.setPosition(armIntake))
                .waitSeconds(1)
                .forward(8.5) // 8.5
                .turn(Math.toRadians(-25)) // -35
                .addTemporalMarker(() -> RightGrip.setPosition(RightGripIntake)) // Left Grip Open
                .waitSeconds(2)
                .back(5)
                .waitSeconds(10)
                .build();

        TrajectorySequence positionCenter = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> Arm.setPosition(armIntake))
                .waitSeconds(0.5)
                .forward(11.5)
                .addTemporalMarker(() -> RightGrip.setPosition(RightGripIntake)) // Left Grip Open
                .waitSeconds(2)
                .back(5)
                .waitSeconds(10)
                .build();

        if (opModeIsActive())   //&& ParkingZone == "None"
        {
            //// TODO ====================== New Implementaion ===============================
            switch (randomization) {
                case LEFT:
                    drive.followTrajectorySequence(positionLeft);
                    break;
                case CENTER:
                    drive.followTrajectorySequence(positionCenter);
                    break;
                case RIGHT:
                    drive.followTrajectorySequence(positionRight);
                    break;
                default:
                    break;

            }

        }
    }
    public void RotateMotorClockPos ( int Pos, double POWER){
        Rotate.setTargetPosition(Pos);
        Rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rotate.setPower(POWER);
    }
}

