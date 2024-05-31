package org.firstinspires.ftc.teamcode;

import com.amarcolini.joos.control.PIDCoefficients;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.drive.PIDSwerveModule;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.hardware.CRServo;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.localization.AngleSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.jetbrains.annotations.NotNull;

@JoosConfig
public class SampleSwerveModule extends PIDSwerveModule {
    public static final PIDCoefficients coeffs = new PIDCoefficients(0.0);
    public final AngleSensor moduleOrientationSensor;
    public final Motor motor;
    public final CRServo servo;

    public SampleSwerveModule(
            AngleSensor moduleOrientationSensor,
            Motor motor,
            CRServo servo
    ) {
        super(coeffs);
        this.moduleOrientationSensor = moduleOrientationSensor;
        this.motor = motor;
        this.servo = servo;
    }

    @Override
    protected void setCorrectedDrivePower(double v) {
        motor.setPower(v);
    }

    @Override
    protected void setCorrectedWheelVelocity(double v, double v1) {
        motor.setDistanceVelocity(v, v1);
    }

    @Override
    public void setModulePower(double v) {
        servo.setPower(v);
    }

    @NotNull
    @Override
    public Angle getModuleOrientation() {
        return moduleOrientationSensor.getAngle();
    }

    @Override
    public double getWheelPosition() {
        return motor.getDistance();
    }
}