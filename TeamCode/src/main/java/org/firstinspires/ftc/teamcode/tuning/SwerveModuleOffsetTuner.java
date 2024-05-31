package org.firstinspires.ftc.teamcode.tuning;

import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.command.Component;
import com.amarcolini.joos.drive.AbstractSwerveDrive;
import com.amarcolini.joos.drive.PIDSwerveModule;
import com.amarcolini.joos.drive.SwerveModule;
import com.amarcolini.joos.geometry.Angle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.AxonAngleSensor;
import org.firstinspires.ftc.teamcode.SampleRobot;
import org.firstinspires.ftc.teamcode.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.SampleSwerveModule;

import java.util.List;

@TeleOp
public class SwerveModuleOffsetTuner extends CommandOpMode {
    @Register
    private SampleRobot robot;

    @Override
    public void preInit() {
        if (robot.drive instanceof Component) unregister((Component) robot.drive);
        assert robot.drive instanceof AbstractSwerveDrive : "Only swerve drives are allowed";
        final AbstractSwerveDrive swerve = (AbstractSwerveDrive) robot.drive;
        for (SwerveModule module : swerve.getModules()) {
            if (module instanceof PIDSwerveModule) ((PIDSwerveModule) module).setModulePower(0.0);
        }
        telem.addLine("Turn all modules to their 0 orientation (pointing forwards).").setRetained(true);
        schedule(true, () -> {
            List<SwerveModule> modules = swerve.getModules();
            List<Angle> orientations = swerve.getModuleOrientations();
            for (int i = 0; i < modules.size(); i++) {
                SwerveModule module = modules.get(i);
                if (module instanceof SampleSwerveModule) {
                    Angle offset = ((SampleSwerveModule) module).moduleOrientationSensor.getOffset();
                    telem.addData("offset for module " + i, orientations.get(i).minus(offset).unaryMinus());
                }
                telem.addData("orientation of module " + i, orientations.get(i));
            }
        });
    }
}