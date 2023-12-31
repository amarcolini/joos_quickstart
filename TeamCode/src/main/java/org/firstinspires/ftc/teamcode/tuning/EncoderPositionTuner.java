package org.firstinspires.ftc.teamcode.tuning;

import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.command.SequentialCommand;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SampleRobot;
import org.firstinspires.ftc.teamcode.tuning.util.EncoderData;
import org.firstinspires.ftc.teamcode.tuning.util.TuningData;

import java.util.Arrays;
import java.util.List;

@TeleOp(group = "Tuning")
@JoosConfig
public class EncoderPositionTuner extends CommandOpMode {
    @Register
    private SampleRobot robot;
    public static int totalTurns = 10;

//    private val headingSensor by getHardware<IMU>("imu").map {
//        getAngleSensor(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP
//        )
//    }

    @Override
    public void preInit() {
        List<EncoderData> encoders = new TuningData(robot.drive).encoders;
        assert !encoders.isEmpty() : "This drive does not support this test.";

        new SequentialCommand(true, Command.of(() ->
                robot.drive.setDrivePower(
                        new Pose2d(0, 0, Angle.rad(-gamepad().p1.getLeftStick().x))
                )
        ).runUntil(gamepad((p1, p2) -> p1.a.or(p1.cross))::isActive), Command.of(() -> {
//            telem.addData("heading", angVels).setRetained(true);
//            telem.addData("left", leftVels).setRetained(true);
//            telem.addData("right", rightVels).setRetained(true);
//            telem.addData("perp", perpVels).setRetained(true);
//            telem.update();
            Angle theta = Angle.circle.times(totalTurns);
            for (EncoderData encoder : encoders) {
                double solution = encoder.encoder.getAsDouble() / theta.radians();
                Vector2d pos = new Vector2d(solution * encoder.phi.sin(), -solution * encoder.phi.cos());
                telem.addData(encoder.name + " solution", solution).setRetained(true);
                telem.addData(encoder.name + " pos", pos).setRetained(true);
            }
        })).onEnd((interrupted) -> {
            robot.drive.setDrivePower(new Pose2d());
            telem.addLine("finished!").setRetained(true);
        }).schedule();
    }
}