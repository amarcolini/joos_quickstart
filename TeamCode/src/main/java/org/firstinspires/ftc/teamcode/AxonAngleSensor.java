package org.firstinspires.ftc.teamcode;

import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.localization.AngleSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.jetbrains.annotations.NotNull;

public class AxonAngleSensor extends AngleSensor {
    public final Angle offset;
    private final AnalogInput input;
    private final boolean reversed;

    public AxonAngleSensor(
            AnalogInput input,
            Angle offset,
            boolean reversed
    ) {
        this.input = input;
        this.offset = offset;
        this.reversed = reversed;
    }

    @NotNull
    @Override
    protected Angle getRawAngle() {
        return Angle.circle.times(input.getVoltage() / input.getMaxVoltage() * (reversed ? 1 : -1)).plus(offset);
    }
}