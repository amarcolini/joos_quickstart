package org.firstinspires.ftc.teamcode.tuning;

import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.hardware.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SampleRobot;

@TeleOp(group = "Tuning")
public class ForwardPushTest extends CommandOpMode {
    @Register
    private SampleRobot robot;

    private double getTicks() {
        return robot.drive.motorGroup.getCurrentPosition();
    }

    @Override
    public void preInit() {
        final double initTicks = getTicks();

        robot.drive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        robot.drive.setDrivePower(new Pose2d());

        schedule(true, () -> {
            double ticks = getTicks() - initTicks;
            telem.addData("Forward Encoder Ticks", ticks);
        });
    }
}