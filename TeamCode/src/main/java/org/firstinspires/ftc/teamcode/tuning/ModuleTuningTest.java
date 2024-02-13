package org.firstinspires.ftc.teamcode.tuning;

import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.command.SequentialCommand;
import com.amarcolini.joos.command.WaitCommand;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.geometry.Angle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SampleRobot;

import java.util.Arrays;

@TeleOp
@JoosConfig
public class ModuleTuningTest extends CommandOpMode {
    @Register
    private SampleRobot robot;

    public static Angle startAngle = Angle.deg(0);
    public static Angle endAngle = Angle.deg(90);

    @Override
    public void preInit() {
        new SequentialCommand(
                Command.of(() -> robot.drive.setModuleOrientations(Arrays.asList(
                        startAngle,
                        startAngle,
                        startAngle,
                        startAngle
                ))),
                new WaitCommand(3.0),
                Command.of(() -> robot.drive.setModuleOrientations(Arrays.asList(
                        endAngle,
                        endAngle,
                        endAngle,
                        endAngle
                ))),
                new WaitCommand(3.0)
        ).repeatForever().requires(robot.drive).schedule();
    }
}