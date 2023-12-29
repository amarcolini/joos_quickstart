package org.firstinspires.ftc.teamcode.tuning;

import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.command.SequentialCommand;
import com.amarcolini.joos.command.TimeCommand;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SampleRobot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

import static com.amarcolini.joos.util.MathUtil.doLinearRegressionNoIntercept;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

@TeleOp(group = "Tuning")
@JoosConfig
public class AngularRampLogger extends CommandOpMode {
    @Register
    private SampleRobot robot;
    public static double totalTime = 5.0;

//    private val headingSensor by getHardware<IMU>("imu").map {
//        getAngleSensor(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP
//        )
//    }

    public static class EncoderData {
        public final String name;
        public final ArrayList<Double> data = new ArrayList<>();
        public final Angle phi;
        public final DoubleSupplier encoder;

        public EncoderData(String name, Angle phi, DoubleSupplier encoder) {
            this.name = name;
            this.encoder = encoder;
            this.phi = phi;
        }
    }

    @Override
    public void preInit() {
        ArrayList<Double> angVels = new ArrayList<>();
        List<EncoderData> encoders = Arrays.asList(
                new EncoderData("leftMotors", Angle.deg(0), () -> robot.drive.leftMotors.getDistance()),
                new EncoderData("rightMotors", Angle.deg(0), () -> robot.drive.rightMotors.getDistance())
//                new EncoderData("left", leftVels, Drivetrain.leftPose.heading),
//                new EncoderData("right", rightVels, Drivetrain.rightPose.heading),
//                new EncoderData("perp", perpVels, Drivetrain.perpPose.heading)
        );

        new SequentialCommand(true, new TimeCommand((t, dt) -> {
            robot.drive.setDrivePower(new Pose2d(0, 0, Angle.rad(t / totalTime)));
            angVels.add(robot.headingSensor.getAngularVelocity().radians());
            for (EncoderData encoder : encoders) {
                encoder.data.add(encoder.encoder.getAsDouble());
            }
            return t > totalTime;
        }), Command.of(() -> {
//            telem.addData("heading", angVels).setRetained(true);
//            telem.addData("left", leftVels).setRetained(true);
//            telem.addData("right", rightVels).setRetained(true);
//            telem.addData("perp", perpVels).setRetained(true);
//            telem.update();
            for (EncoderData encoder : encoders) {
                double solution = doLinearRegressionNoIntercept(angVels, encoder.data);
                Vector2d pos = new Vector2d(solution * encoder.phi.sin(), -solution * encoder.phi.cos());
                telem.addData(encoder.name + " solution", solution).setRetained(true);
                telem.addData(encoder.name + " pos", pos).setRetained(true);
            }
        })).onEnd((interrupted) -> {
            robot.drive.setDrivePower(

                    new Pose2d());
            telem.addLine("finished!").setRetained(true);
        }).schedule();
    }
}