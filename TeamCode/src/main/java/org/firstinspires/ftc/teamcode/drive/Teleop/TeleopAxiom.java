package org.firstinspires.ftc.teamcode.drive.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@TeleOp(group = "Teleop_Axiom")
@Config
public class TeleopAxiom extends LinearOpMode {

    static Servo Drone;
    static Servo Arm;
    public static Servo LeftGrip;
    public static Servo RightGrip;
    public static DcMotorEx Lift;
    public static DcMotorEx Rotate;

    // ============================== Drone Shoot Value ============================
<<<<<<< HEAD
    public static double droneShoot=0.3, droneRest=0.6;
=======
    public static double droneShoot=0.3, droneRest=0.7;
>>>>>>> d51de313125352f74323feb672024258b047c90e

    // ============================= Arm Intake and Drop Value =============================
    public static double armIntake = 0.22, armDrop = 0.85;

    // ================================= LeftGrip/RightGrip Values ===========================

    public static double LeftGripIntake = 0.6, LeftGripOuttake = 1, RightGripIntake = 0.4, RightGripOuttake = 0;

    // ================================= Lift pos ============================================

     public static  int LiftClosepos = 0, LiftExtendpos = 1705, RotateIntakepos = -10, RotateDroppos = 1478, Hangpos = 200, HangposLift=0;


    @Override
    public void runOpMode() throws InterruptedException {


        SampleTankDrive drive = new SampleTankDrive(hardwareMap); // Using Sample Tank Drive to drive the robot

        // Servo Mechanism
        Drone = hardwareMap.get(Servo.class, "drone");
        Arm = hardwareMap.get(Servo.class, "arm");
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

        if(opModeInInit()){
            Drone(droneRest);
            LeftGrip.setPosition(LeftGripOuttake);
            RightGrip.setPosition(RightGripOuttake);
        }
        waitForStart(); // Waiting for bot to Start




        // This Condition will run Until the OPMode(Start) is Active
        while(opModeIsActive())
        {

            // ==================== ARM MECHANISM =========================================

            if(gamepad1.right_bumper)
            {
                // Arm In Intake Pos
                RotateMotorAntiClockPos(RotateIntakepos, 0.3);
                Gripper(LeftGripIntake, RightGripIntake);
<<<<<<< HEAD
=======
                LiftMotorDown(LiftClosepos, 0.5); // Extra added
>>>>>>> d51de313125352f74323feb672024258b047c90e
            }
             if(gamepad1.left_bumper)
            {
                // Arm In Dropping Pos
                RotateMotorClockPos(RotateDroppos, 0.3);
                ArmHand(armDrop);
            }

             if(gamepad1.x){
                 ArmHand(armIntake);

             }
             if(gamepad1.y){
                 ArmHand(armDrop);
             }

            // ======================== GRIP Motion ============================================

             if(gamepad1.right_trigger>0)
            {
                // Left Grip Close
                Gripper(LeftGripIntake, RightGripIntake);
            }
             if(gamepad1.left_trigger>0)
            {
                // Right Grip Close
                Gripper(LeftGripOuttake, RightGripOuttake);
            }

            else if(gamepad1.a)
            {
                // Left Grip Open
                LeftGripper(LeftGripIntake);
            }
            else if(gamepad1.b)
            {
                // Right Grip Open
                RightGripper(RightGripIntake);
            }

            // ========================= Slider Motion ===========================================

             if(gamepad1.dpad_up){
                // Slider UP set to pos
                LiftMotorUp(LiftExtendpos, 0.5);
            }

             if(gamepad1.dpad_down){
                // Slider Down set to pos
                LiftMotorDown(LiftClosepos,0.5);
            }

            else if(gamepad2.right_bumper){
                // Slider UP Linear decrement by 10
                LiftMotorUp( 0.5);
            }

            else if(gamepad2.left_bumper){
                // Slider Down Linear Increment by 10
                LiftMotorDown(0.5);
            }

            // =============================== Rotate Motion ======================================

             if(gamepad1.dpad_up){
                // Rotate Clock
                RotateMotorClockPos(RotateDroppos, 0.5);
            }
             if(gamepad1.dpad_right){
//                 Rotate Anti Clock
                RotateMotorAntiClockPos(RotateIntakepos, 0.5);

            }
            else if(gamepad2.dpad_up){
                // Rotate Clock
                RotateMotorClock(0.5);
            }
            else if(gamepad2.dpad_down){
//                 Rotate Anti Clock
                RotateMotorAntiClock(0.5);

            }

//             =========================== Hanging Pos ============================================

             else if (gamepad2.a) {
                 RotateMotorAntiClockPos(Hangpos, 0.5);
                 LiftMotorDown(HangposLift, 0.5);
             }

            // ======================= Drone Shoot ================================================

            else if(gamepad2.y){
                // Drone Shoot
                Drone(droneShoot);
            } else if (gamepad2.x) {
                 RotateMotorClockPos(753, 0.5);
                 LiftMotorUp(1400, 0.5);
             }


            // ============================ Driving the Robot ==================================

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y/2,
                            0,
                            -gamepad1.right_stick_x/2
                    )
            );
            drive.update();
            telemetry.addData("Motor R Encoder Value", Rotate.getCurrentPosition());
            telemetry.addData("Motor L Encoder Value", Lift.getCurrentPosition());
            telemetry.update();
        }

    }


    //// TODO -> Functions Used
    public static void Gripper(double LeftGripIntakePos, double RightGripIntake){
        LeftGrip.setPosition(LeftGripIntakePos);
        RightGrip.setPosition(RightGripIntake);
    }

    public static void LeftGripper(double LeftGRipPOs){
        LeftGrip.setPosition(LeftGRipPOs);
    }

    public static void RightGripper(double RightGRipPOs){
        RightGrip.setPosition(RightGRipPOs);
    }

    public static void Drone(double DroneShootPos){
        Drone.setPosition(DroneShootPos);
    }

    public static void ArmHand(double ArmIntakePos){
        Arm.setPosition(ArmIntakePos);
    }

    public static void RotateMotorClock(double POWER) {
        Rotate.setTargetPosition(Rotate.getTargetPosition()+10);
        Rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rotate.setPower(POWER);
    }
    public static void RotateMotorAntiClock(double POWER) {
        Rotate.setTargetPosition(Rotate.getTargetPosition()-10);
        Rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rotate.setPower(POWER);
    }
    public void RotateMotorClockPos(int Pos, double POWER) {
        Rotate.setTargetPosition(Pos);
        Rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rotate.setPower(POWER);
    }
    public static void RotateMotorAntiClockPos(int Pos, double POWER) {
        Rotate.setTargetPosition(Pos);
        Rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rotate.setPower(POWER);
    }

    // This Function is used to set Lift Motor Power
    public static void LiftMotorUp(double POWER) {
        Lift.setTargetPosition(Lift.getTargetPosition()+10);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(POWER);
    }
    public static void LiftMotorDown(double POWER) {
        Lift.setTargetPosition(Lift.getTargetPosition()-10);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(POWER);
    }
    public static void LiftMotorUp(int Pos, double POWER) {
        Lift.setTargetPosition(Pos);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(POWER);
    }
    public static void LiftMotorDown(int Pos, double POWER) {
        Lift.setTargetPosition(Pos);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(POWER);
    }
}
