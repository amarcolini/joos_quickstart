package org.firstinspires.ftc.teamcode.tuning;

import com.amarcolini.joos.command.*;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.drive.DriveSignal;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.geometry.Vector2d;
import com.amarcolini.joos.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SampleRobot;

@TeleOp(group = "Tuning")
@JoosConfig
public class ManualFeedforwardTuner extends CommandOpMode {
    @Register
    private SampleRobot robot;
    public static double distance = 48.0;

    private double targetVelocity = 0.0;

    @Override
    public void preInit() {
        Trajectory forwardTrajectory = robot.drive.trajectoryBuilder()
                .forward(distance)
                .build();
        Trajectory backwardTrajectory = robot.drive.trajectoryBuilder(forwardTrajectory.end())
                .back(distance)
                .build();

        targetVelocity = 0.0;

        Command tuningCommand = new SequentialCommand(
                true,
                new TimeCommand((t, dt) -> {
                    Pose2d vel = forwardTrajectory.velocity(t);
                    targetVelocity = vel.x;
                    robot.drive.setDriveSignal(new DriveSignal(vel, forwardTrajectory.acceleration(t)));
                    return t >= forwardTrajectory.duration();
                }).onEnd((interrupted) -> robot.drive.setDriveSignal(new DriveSignal())),
                new WaitCommand(0.5),
                new TimeCommand((t, dt) -> {
                    Pose2d vel = backwardTrajectory.velocity(t);
                    targetVelocity = vel.x;
                    robot.drive.setDriveSignal(new DriveSignal(vel, backwardTrajectory.acceleration(t)));
                    return t >= backwardTrajectory.duration();
                }).onEnd((interrupted) -> robot.drive.setDriveSignal(new DriveSignal())),
                new WaitCommand(0.5)
        ).repeatForever().requires(robot.drive);

        Command resettingCommand = Command.of(() -> {
            Vector2d leftStick = gamepad().p1.getLeftStick();
            Vector2d rightStick = gamepad().p1.getRightStick();
            robot.drive.setDrivePower(
                    new Pose2d(
                            -leftStick.y,
                            -leftStick.x,
                            Angle.rad(-rightStick.x)
                    )
            );
        }).runForever().requires(robot.drive);

        Command loggingCommand = new InstantCommand(() -> {
            Pose2d actualVelocity = robot.drive.getPoseVelocity();
            if (actualVelocity == null) return;
            telem.addData("targetVelocity", targetVelocity);
            telem.addData("actualVelocity", actualVelocity.x);
        }).repeatForever();
        loggingCommand.runBlocking();

        schedule(tuningCommand, loggingCommand);
        map(gamepad().p1.y0::isJustActivated, resettingCommand);
        map(gamepad().p1.b0::isJustActivated, tuningCommand);
    }
}