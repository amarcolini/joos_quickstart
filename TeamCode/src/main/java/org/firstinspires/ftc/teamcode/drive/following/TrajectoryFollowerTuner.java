package org.firstinspires.ftc.teamcode.drive.following;

import androidx.annotation.NonNull;

import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.SampleBot;

@JoosConfig
@Autonomous(group = "following")
public class TrajectoryFollowerTuner extends CommandOpMode {
    public static double DISTANCE = 30;

    @Override
    public void preInit() {
        SampleBot robot = registerRobot(new SampleBot());
        Pose2d start = new Pose2d(-DISTANCE * 0.5, -DISTANCE * 0.5);
        robot.drive.setPoseEstimate(start);

        schedule(
                robot.drive.followTrajectory(
                        robot.drive.trajectoryBuilder(start)
                                .forward(DISTANCE)
                                .turn(Angle.deg(90))
                                .forward(DISTANCE)
                                .turn(Angle.deg(90))
                                .forward(DISTANCE)
                                .turn(Angle.deg(90))
                                .forward(DISTANCE)
                                .turn(Angle.deg(90))
                                .build()
                ).repeatForever()
        );
    }
}