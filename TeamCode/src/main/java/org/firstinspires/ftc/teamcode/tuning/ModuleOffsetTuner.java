package org.firstinspires.ftc.teamcode.tuning;

import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.drive.PIDSwerveModule;
import com.amarcolini.joos.drive.SwerveModule;
import com.amarcolini.joos.geometry.Angle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SampleRobot;

import java.util.List;

@TeleOp
public class ModuleOffsetTuner extends CommandOpMode {
    @Register
    private SampleRobot robot;

    @Override
    public void preInit() {
        unregister(robot.drive);
        for (SwerveModule module : robot.drive.getModules()) {
            if (module instanceof PIDSwerveModule) ((PIDSwerveModule) module).setModulePower(0.0);
        }
        schedule(true, () -> {
            List<Angle> orientations = robot.drive.getModuleOrientations();
            for (int i = 0; i < orientations.size(); i++) {
                telem.addData("module " + i, orientations.get(i));
            }
        });
    }
}