package org.firstinspires.ftc.teamcode.drive.following;

import androidx.annotation.NonNull;

import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.FunctionalCommand;
import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.command.SequentialCommand;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.drive.DriveSignal;
import com.amarcolini.joos.followers.GVFFollower;
import com.amarcolini.joos.followers.PathFollower;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.geometry.Vector2d;
import com.amarcolini.joos.path.Path;
import com.amarcolini.joos.path.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.SampleBot;

@JoosConfig
@Autonomous(group = "following")
public class PathFollowerTuner extends CommandOpMode {
    public static double DISTANCE = 30;
    public static GVFFollower follower = new GVFFollower(
            40.0, 40.0,
            new Pose2d(0.5, 0.5, Angle.deg(0.5)), 0.1, 2
    );
    private SampleBot robot;

    @Override
    public void preInit() {
        robot = registerRobot(new SampleBot());
        Pose2d start = new Pose2d(-DISTANCE * 0.5, -DISTANCE * 0.5);
        robot.drive.setPoseEstimate(start);

        Path forward = new PathBuilder(start)
                .splineTo(new Vector2d(DISTANCE * 0.5, DISTANCE * 0.5), 0)
                .build();

        Path backward = new PathBuilder(forward.end(), true)
                .splineTo(start.vec(), 0)
                .build();

        schedule(new SequentialCommand(
                followPath(forward),
                followPath(backward)
        ).repeatForever());
    }

    private Command followPath(Path path) {
        return new FunctionalCommand(
                () -> follower.followPath(path),
                () -> follower.update(robot.drive.getPoseEstimate()),
                interrupted -> robot.drive.setDriveSignal(new DriveSignal()),
                () -> !follower.isFollowing(),
                true, robot.drive
        );
    }
}
