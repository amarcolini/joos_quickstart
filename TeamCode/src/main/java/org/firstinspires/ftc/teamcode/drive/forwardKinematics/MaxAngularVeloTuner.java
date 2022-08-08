package org.firstinspires.ftc.teamcode.drive.forwardKinematics;

import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.command.WaitCommand;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.util.MathUtil;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SampleBot;

import java.util.Objects;

@JoosConfig
@Autonomous(group = "forwardKinematics")
public class MaxAngularVeloTuner extends CommandOpMode {
    public static double RUNTIME = 4;
    private SampleBot robot;

    private Angle maxAngVel = new Angle();
    @Override
    public void preInit() {
        robot = registerRobot(new SampleBot());
        telemetry.addLine("Your robot will turn at full speed for " + RUNTIME + " seconds.");
        telemetry.addLine("Make sure you have enough room for the robot to move.");
        schedule(
                new WaitCommand(RUNTIME)
                        .onInit(() -> robot.drive.setDrivePower(
                                new Pose2d(0, 0, 1)
                        ))
                        .onExecute(() -> {
                            Angle vel = Objects.requireNonNull(robot.drive.getPoseVelocity(), "getPoseVelocity() must not return null.").heading;
                            maxAngVel = MathUtil.max(vel, maxAngVel);
                        })
                        .onEnd(interrupted -> {
                            robot.drive.setDrivePower(new Pose2d(0));
                            telemetry.setAutoClear(false);
                            telemetry.addData("Maximum Angular Velocity (deg)", maxAngVel.degrees());
                            telemetry.addData("Maximum Angular Velocity (rad)", maxAngVel.radians());
                        })
                        .requires(robot.drive)
        );
    }
}