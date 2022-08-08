package org.firstinspires.ftc.teamcode.drive.forwardKinematics;

import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.command.SequentialCommand;
import com.amarcolini.joos.command.TimeCommand;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.drive.DriveSignal;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.profile.MotionProfile;
import com.amarcolini.joos.profile.MotionProfileGenerator;
import com.amarcolini.joos.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SampleBot;

@JoosConfig
@Autonomous(group = "forwardKinematics")
public class MotorVelocityTuner extends CommandOpMode {
    public static double DISTANCE = 30;
    private boolean usingEncoder = true;
    private SampleBot robot;

    @Override
    public void preInit() {
        robot = registerRobot(new SampleBot());
        telemetry.addLine("This OpMode is for tuning your motor velocity control. Press A to toggle between motor encoders and no motor encoders.").setRetained(true);
        telemetry.addData("targetVelocity", 0.0).setRetained(true);
        telemetry.addData("measuredVelocity", 0.0).setRetained(true);
        map(gamepad().p1.a::justActivated, () -> usingEncoder = !usingEncoder);
        schedule(() -> telemetry.addData("Using encoders", usingEncoder), true);
    }

    private Command followMotionProfile(MotionProfile profile) {
        return new TimeCommand((t, dt) -> {
            MotionState state = profile.get(t);
            robot.drive.setDriveSignal(new DriveSignal(new Pose2d(state.v), new Pose2d(state.a)));
            telemetry.addData("mode", "Tuning");
            telemetry.addData("targetVelocity", state.v);
            telemetry.addData("measuredVelocity", robot.drive.getPoseVelocity().x);
            return t > profile.duration();
        });
    }

    @Override
    public void preStart() {
        telemetry.clearAll();
        cancelAll();
        unmap(gamepad().p1.a::justActivated);

        if (usingEncoder) robot.drive.setRunMode(Motor.RunMode.RUN_USING_ENCODER);
        else robot.drive.setRunMode(Motor.RunMode.RUN_WITHOUT_ENCODER);

        final Motor.RunMode runMode = robot.drive.motors.getRunMode();

        MotionProfile forward = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0.0, 0.0, 0.0),
                new MotionState(DISTANCE, 0.0, 0.0),
                robot.drive.getConstraints().maxVel, robot.drive.getConstraints().maxAccel, 0.0
        );
        MotionProfile backward = forward.flipped();

        Command tuning = new SequentialCommand(
                followMotionProfile(forward),
                followMotionProfile(backward)
        ).repeatForever().requires(robot.drive);

        Command driveControl = Command.of(() -> {
                    telemetry.addData("mode", "Drive Control");
                    robot.drive.setWeightedDrivePower(new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    ));
                }).runForever().requires(robot.drive)
                .onInit(() -> robot.drive.setRunMode(Motor.RunMode.RUN_WITHOUT_ENCODER))
                .onEnd(interrupted -> robot.drive.setRunMode(runMode));

        map(() -> gamepad().p1.b.justActivated() && tuning.isScheduled(), driveControl);
        map(() -> gamepad().p1.y.justActivated() && driveControl.isScheduled(), tuning);
        schedule(tuning);
    }
}
