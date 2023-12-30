package org.firstinspires.ftc.teamcode.tuning;

import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SampleRobot;

import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

@TeleOp(group = "Tuning")
public class MotorDirectionDebugger extends CommandOpMode {
    @Register
    private SampleRobot robot;

    @Override
    public void preInit() {
        List<Function<GamepadEx, Boolean>> buttonMap = Arrays.asList(
                (p1) -> p1.x.or(p1.square).isActive(),
                (p1) -> p1.a.or(p1.cross).isActive(),
                (p1) -> p1.b.or(p1.circle).isActive(),
                (p1) -> p1.y.or(p1.triangle).isActive()
        );

        schedule(true, () -> {
            for (int i = 0; i < robot.drive.motorGroup.size(); i++) {
                robot.drive.motorGroup.get(i).setPower(
                        buttonMap.get(i).apply(gamepad().p1) ? 1.0 : 0.0
                );
            }
        });
    }
}