package org.firstinspires.ftc.teamcode.tuning.util;

import static com.amarcolini.joos.command.CommandScheduler.telem;

import com.amarcolini.joos.command.Component;
import com.amarcolini.joos.command.FollowPathCommand;
import com.amarcolini.joos.drive.Drive;
import com.amarcolini.joos.followers.PathFollower;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.path.Path;

import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;

public class DashboardPathCommand<T extends Drive & Component> extends FollowPathCommand<T> {
    public DashboardPathCommand(@NotNull Path path, @NotNull PathFollower pathFollower, @NotNull T driveComponent) {
        super(path, pathFollower, driveComponent);
    }

    private final ArrayList<Pose2d> poseHistory = new ArrayList<>();
    public String pathColor = "#4CAF50";
    public String robotColor = "#3F51B5";
    public int poseHistoryLimit = 100;

    @Override
    public void postExecute() {
        poseHistory.add(driveComponent.getLocalizer().getPoseEstimate());
        if (poseHistoryLimit > -1 && poseHistory.size() > poseHistoryLimit)
            poseHistory.remove(0);
        telem.drawSampledPath(path, pathColor);
        telem.drawPoseHistory(poseHistory, robotColor);
        telem.fieldOverlay().setStrokeWidth(1);
        telem.drawRobot(driveComponent.getLocalizer().getPoseEstimate(), robotColor);
    }

    @Override
    public void postInit() {
        telem.drawSampledPath(path, pathColor);
    }
}