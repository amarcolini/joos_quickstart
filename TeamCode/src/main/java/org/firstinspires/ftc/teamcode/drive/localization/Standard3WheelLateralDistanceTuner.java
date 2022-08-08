package org.firstinspires.ftc.teamcode.drive.localization;

import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.hardware.drive.Standard3WheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.SampleBot;

@JoosConfig
@Autonomous(group = "localization")
public class Standard3WheelLateralDistanceTuner extends CommandOpMode {
    public static int NUM_TURNS = 10;
    private Standard3WheelLocalizer localizer;
    private SampleBot robot;

    private Angle totalRotation = new Angle();
    private Angle lastHeading = new Angle();
    @Override
    public void preInit() {
        robot = registerRobot(new SampleBot());
        if (robot.drive.getLocalizer() instanceof Standard3WheelLocalizer) {
            localizer = (Standard3WheelLocalizer) robot.drive.getLocalizer();
        } else {
            RobotLog.setGlobalErrorMsg("You aren't using a Standard3WheelLocalizer for your drive, " +
                    "so you don't need to use this OpMode. If you are, then maybe you haven't set it.");
        }

        schedule(Command.of(() -> {
            Angle heading = robot.drive.getPoseEstimate().heading;
            totalRotation = totalRotation.plus(heading.minus(lastHeading).normDelta());
            lastHeading = heading;
            telemetry.addLine("Total Rotation (deg): " + totalRotation.degrees());
            telemetry.addLine("Raw Heading (deg): " + heading.degrees());

            robot.drive.setWeightedDrivePower(new Pose2d(
                    0,
                    0,
                    gamepad().p1.getRightStick().x
            ));
        }).runUntil(gamepad().p1.y::justActivated).then(() -> {
            telemetry.addLine("Localizer's total rotation: " + totalRotation.toString()).setRetained(true);
            telemetry.addLine("Effective LATERAL_DISTANCE: " +
                    (totalRotation.div(Angle.rad(NUM_TURNS * Math.PI * 2)) * localizer.lateralDistance)).setRetained(true);
        }));

        telemetry.addLine("Ready!");
    }
}
