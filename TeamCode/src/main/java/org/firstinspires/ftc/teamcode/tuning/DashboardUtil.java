package org.firstinspires.ftc.teamcode.tuning;

import com.amarcolini.joos.command.CommandScheduler;
import com.amarcolini.joos.dashboard.SuperTelemetry;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.hardware.drive.DriveComponent;
import com.amarcolini.joos.hardware.drive.DriveTrajectoryFollower;
import com.amarcolini.joos.trajectory.Trajectory;

import java.util.ArrayList;

public class DashboardUtil {
    private final DriveTrajectoryFollower drive;

    public DashboardUtil(DriveTrajectoryFollower drive) {
        this.drive = drive;
    }

    private final ArrayList<Pose2d> poseHistory = new ArrayList<>();
    public String pathColor = "#4CAF50";
    public String turnColor = "#7c4dff";
    public String waitColor = "#dd2c00";
    public String robotColor = "#3F51B5";
    public int poseHistoryLimit = 100;

    public void update() {
        SuperTelemetry telem = CommandScheduler.telemetry;
        Trajectory trajectory = drive.getCurrentTrajectory();
        if (trajectory != null) {
            poseHistory.add(drive.getPoseEstimate());
            if (poseHistoryLimit > -1 && poseHistory.size() > poseHistoryLimit)
                poseHistory.remove(0);
            telem.drawSampledTrajectory(trajectory, pathColor, turnColor, waitColor);
            telem.drawPoseHistory(poseHistory, robotColor);
        }
        telem.fieldOverlay().setStrokeWidth(1);
        telem.drawRobot(drive.getPoseEstimate(), robotColor);
    }
}