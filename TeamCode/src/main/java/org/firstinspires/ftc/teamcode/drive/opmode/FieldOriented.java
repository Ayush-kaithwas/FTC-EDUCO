package org.firstinspires.ftc.teamcode.drive.opmode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;


@Disabled
@TeleOp(group = "Field Oriented")
public class FieldOriented extends LinearOpMode {
    public double THROTTLE = 0.8, TURN = 0.8, HEADING = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {

            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(Math.pow(Range.clip(-gamepad1.left_stick_y, -1, 1), 3), Math.pow(Range.clip(-gamepad1.left_stick_x, -1, 1), 3)).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(input.getX() * THROTTLE, input.getY() * TURN, -gamepad1.right_stick_x * HEADING)
            );
            drive.update();

        }


    }
}
