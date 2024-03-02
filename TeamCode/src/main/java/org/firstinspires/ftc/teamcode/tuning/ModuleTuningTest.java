package org.firstinspires.ftc.teamcode.tuning;

import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.command.SequentialCommand;
import com.amarcolini.joos.command.WaitCommand;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.geometry.Angle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SampleRobot;

@TeleOp
@JoosConfig
public class ModuleTuningTest extends CommandOpMode {
    @Register
    private SampleRobot robot;
    public static Angle startAngle = Angle.deg(0);
    public static Angle endAngle = Angle.deg(60);
    public static double delay = 3.0;
    public static int moduleIndex = 0;

    @Override
    public void preInit() {
        new SequentialCommand(
                Command.of(() -> robot.drive.getModules().get(moduleIndex).setModuleOrientation(startAngle)),
                new WaitCommand(delay),
                Command.of(() -> robot.drive.getModules().get(moduleIndex).setModuleOrientation(endAngle)),
                new WaitCommand(delay)
        ).repeatForever().requires(robot.drive).schedule();
    }
}