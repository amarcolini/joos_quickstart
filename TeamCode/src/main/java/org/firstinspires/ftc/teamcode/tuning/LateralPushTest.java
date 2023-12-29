package org.firstinspires.ftc.teamcode.tuning;

import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.drive.AbstractMecanumDrive;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.hardware.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SampleRobot;

import java.util.List;

@TeleOp(group = "Tuning")
public class LateralPushTest extends CommandOpMode {
    @Register
    private SampleRobot robot;

    private double getTicks() {
        List<Integer> positions = robot.drive.motorGroup.getPositions();
        return 0.25 * (-positions.get(0) + positions.get(1) - positions.get(2) + positions.get(3));
    }

    @Override
    public void preInit() {
        assert robot.drive instanceof AbstractMecanumDrive : "This test is only useful for mecanum drives.";

        double initTicks = getTicks();

        robot.drive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        robot.drive.setDrivePower(new Pose2d());

        schedule(true, () -> {
            double ticks = getTicks() - initTicks;
            telem.addData("Lateral Encoder Ticks", ticks);
        });
    }
}