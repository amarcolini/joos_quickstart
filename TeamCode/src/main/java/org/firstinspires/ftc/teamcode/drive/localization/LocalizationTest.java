package org.firstinspires.ftc.teamcode.drive.localization;

import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SampleBot;

@Autonomous(group = "localization")
public class LocalizationTest extends CommandOpMode {
    private SampleBot robot;

    @Override
    public void preInit() {
        robot = registerRobot(new SampleBot());

        schedule(() -> {
            robot.drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));
            Pose2d poseEstimate = robot.drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.x);
            telemetry.addData("y", poseEstimate.y);
            telemetry.addData("heading", poseEstimate.heading);
        }, true);

        telemetry.addLine("ready!");
    }
}
