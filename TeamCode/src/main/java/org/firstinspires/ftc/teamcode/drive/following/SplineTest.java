package org.firstinspires.ftc.teamcode.drive.following;

import androidx.annotation.NonNull;

import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SampleBot;

import kotlin.jvm.functions.Function0;

@JoosConfig
@Autonomous(group = "following")
public class SplineTest extends CommandOpMode {
    public static double DISTANCE = 30;

    @Override
    public void preInit() {
        SampleBot robot = registerRobot(new SampleBot());
        schedule(
                robot.drive.followTrajectory(
                        robot.drive.trajectoryBuilder(new Pose2d())
                                .splineTo(new Vector2d(DISTANCE, DISTANCE), 0)
                                .build()
                ).then(
                        robot.drive.followTrajectory(
                                robot.drive.trajectoryBuilder(new Pose2d(DISTANCE, DISTANCE), true)
                                        .splineTo(new Vector2d(), 180)
                                        .build()
                        )
                ).thenStopOpMode()
        );
    }
}