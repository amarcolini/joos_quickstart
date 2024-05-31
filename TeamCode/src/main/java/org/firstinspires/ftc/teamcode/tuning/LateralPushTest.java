package org.firstinspires.ftc.teamcode.tuning;

import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.drive.AbstractMecanumDrive;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.hardware.drive.DriveComponent;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SampleRobot;
import org.firstinspires.ftc.teamcode.tuning.util.TuningData;

import java.util.List;

@TeleOp(group = "Tuning")
public class LateralPushTest extends CommandOpMode {
    @Register
    private SampleRobot robot;

    @Override
    public void preInit() {
        final TuningData data = new TuningData(robot.drive);
        assert data.lateralTicks != null : "This drive does not support this test.";
        double initTicks = data.lateralTicks.getAsDouble();

        if (robot.drive instanceof DriveComponent) ((DriveComponent) robot.drive).setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        robot.drive.setDrivePower(new Pose2d());

        schedule(true, () -> {
            double ticks = data.lateralTicks.getAsDouble() - initTicks;
            telem.addData("Lateral Encoder Ticks", ticks);
        });
    }
}