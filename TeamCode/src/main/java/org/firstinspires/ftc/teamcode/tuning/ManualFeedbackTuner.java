package org.firstinspires.ftc.teamcode.tuning;

import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.command.SequentialCommand;
import com.amarcolini.joos.command.WaitCommand;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SampleRobot;

@TeleOp
@JoosConfig
public class ManualFeedbackTuner extends CommandOpMode {
    @Register
    private SampleRobot robot;
    public static double distance = 48.0;

    @Override
    public void preInit() {
        robot.drive.setPoseEstimate(new Pose2d());
        Trajectory forwardTrajectory = robot.drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(distance, 0.0, Angle.deg(180)))
                .build();
        Trajectory backwardTrajectory =
                robot.drive.trajectoryBuilder(forwardTrajectory.end())
                        .lineToSplineHeading(new Pose2d())
                        .build();

        new SequentialCommand(
                false,
                robot.drive.followTrajectory(forwardTrajectory),
                new WaitCommand(0.5),
                robot.drive.followTrajectory(backwardTrajectory),
                new WaitCommand(0.5)
        ).repeatForever().schedule();
    }
}