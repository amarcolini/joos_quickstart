package org.firstinspires.ftc.teamcode.drive.following;
import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.command.SequentialCommand;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.SampleBot;

@JoosConfig
@Autonomous(group = "following")
public class BackAndForth extends CommandOpMode {
    public static double DISTANCE = 30;

    @Override
    public void preInit() {
        SampleBot robot = registerRobot(new SampleBot());
        Trajectory forward = robot.drive.trajectoryBuilder(new Pose2d()).forward(DISTANCE).build();
        Trajectory backward = robot.drive.trajectoryBuilder(forward.end()).back(DISTANCE).build();
        robot.drive.setPoseEstimate(new Pose2d());

        schedule(new SequentialCommand(
                robot.drive.followTrajectory(forward),
                robot.drive.followTrajectory(backward)
        ).repeatForever());
    }
}
