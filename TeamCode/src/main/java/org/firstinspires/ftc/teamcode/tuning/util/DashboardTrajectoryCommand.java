package org.firstinspires.ftc.teamcode.tuning.util;

import static com.amarcolini.joos.command.CommandScheduler.telem;

import com.amarcolini.joos.command.Component;
import com.amarcolini.joos.drive.Drive;
import com.amarcolini.joos.followers.TrajectoryFollower;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.command.FollowTrajectoryCommand;
import com.amarcolini.joos.trajectory.Trajectory;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;

public class DashboardTrajectoryCommand<T extends Drive & Component> extends FollowTrajectoryCommand<T> {
    public DashboardTrajectoryCommand(@NotNull Trajectory trajectory, @NotNull TrajectoryFollower trajectoryFollower, @NotNull T driveComponent) {
        super(trajectory, trajectoryFollower, driveComponent);
    }

    private final ArrayList<Pose2d> poseHistory = new ArrayList<>();
    public String pathColor = "#4CAF50";
    public String turnColor = "#7c4dff";
    public String waitColor = "#dd2c00";
    public String robotColor = "#3F51B5";
    public int poseHistoryLimit = 100;

    @Override
    public void postExecute() {
        poseHistory.add(driveComponent.getLocalizer().getPoseEstimate());
        if (poseHistoryLimit > -1 && poseHistory.size() > poseHistoryLimit)
            poseHistory.remove(0);
        telem.drawSampledTrajectory(trajectory, pathColor, turnColor, waitColor);
        telem.drawPoseHistory(poseHistory, robotColor);
        telem.fieldOverlay().setStrokeWidth(1);
        telem.drawRobot(driveComponent.getLocalizer().getPoseEstimate(), robotColor);
    }

    @Override
    public void postInit() {
        telem.drawSampledTrajectory(trajectory, pathColor, turnColor, waitColor);
    }
}