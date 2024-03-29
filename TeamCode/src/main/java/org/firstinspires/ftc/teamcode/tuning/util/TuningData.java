package org.firstinspires.ftc.teamcode.tuning.util;

import androidx.annotation.Nullable;
import com.amarcolini.joos.drive.Drive;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.localization.Localizer;
import com.amarcolini.joos.localization.MecanumLocalizer;
import com.amarcolini.joos.localization.ThreeTrackingWheelLocalizer;
import com.amarcolini.joos.localization.TwoTrackingWheelLocalizer;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class TuningData {
    public final Drive drive;
    public final @Nullable DoubleSupplier forwardTicks;
    public final @Nullable DoubleSupplier lateralTicks;
    public final List<EncoderData> encoders;

    public TuningData(Drive drive) {
        this.drive = drive;
        Localizer localizer = drive.getLocalizer();

        if (localizer instanceof MecanumLocalizer) {
            MecanumLocalizer mecanumLocalizer = (MecanumLocalizer) localizer;
            forwardTicks = () -> {
                List<Double> wheelPositions = mecanumLocalizer.getWheelPositions.invoke();
                return (wheelPositions.get(0) + wheelPositions.get(1) + wheelPositions.get(2) + wheelPositions.get(3)) / 4;
            };
            lateralTicks = () -> {
                List<Double> wheelPositions = mecanumLocalizer.getWheelPositions.invoke();
                return (-wheelPositions.get(0) + wheelPositions.get(1) - wheelPositions.get(2) + wheelPositions.get(3)) / 4;
            };
            encoders = Arrays.asList(
                    new EncoderData("leftMotors", Angle.rad(0), () -> {
                        List<Double> wheelPositions = mecanumLocalizer.getWheelPositions.invoke();
                        return (wheelPositions.get(0) + wheelPositions.get(1)) / 2;
                    }),
                    new EncoderData("rightMotors", Angle.rad(0), () -> {
                        List<Double> wheelPositions = mecanumLocalizer.getWheelPositions.invoke();
                        return (wheelPositions.get(2) + wheelPositions.get(3)) / 2;
                    })
            );
        } else if (localizer instanceof ThreeTrackingWheelLocalizer) {
            ThreeTrackingWheelLocalizer trackingWheelLocalizer = (ThreeTrackingWheelLocalizer) localizer;
            List<Pose2d> wheelPoses = trackingWheelLocalizer.wheelPoses;
            String[] names;
            if (wheelPoses.get(0).heading.epsilonEquals(
                    Angle.deg(0)
            ) && wheelPoses.get(1).heading.epsilonEquals(Angle.deg(0)) &&
                    wheelPoses.get(2).heading.epsilonEquals(Angle.deg(90))) {
                names = new String[]{"leftEnc", "rightEnc", "perpEnc"};
            } else names = new String[]{"enc0", "enc1", "enc2"};
            forwardTicks = () -> mapTicks(trackingWheelLocalizer.getWheelPositions(),
                    wheelPoses, (p) -> p.heading.cos());
            lateralTicks = () -> mapTicks(trackingWheelLocalizer.getWheelPositions(),
                    wheelPoses, (p) -> p.heading.sin());
            encoders = Arrays.asList(
                    new EncoderData(names[0], wheelPoses.get(0).heading, () -> trackingWheelLocalizer.getWheelPositions().get(0)),
                    new EncoderData(names[1], wheelPoses.get(1).heading, () -> trackingWheelLocalizer.getWheelPositions().get(1)),
                    new EncoderData(names[2], wheelPoses.get(2).heading, () -> trackingWheelLocalizer.getWheelPositions().get(2))
            );
        } else if (localizer instanceof TwoTrackingWheelLocalizer) {
            TwoTrackingWheelLocalizer trackingWheelLocalizer = (TwoTrackingWheelLocalizer) localizer;
            List<Pose2d> wheelPoses = trackingWheelLocalizer.wheelPoses;
            String[] names;
            if (wheelPoses.get(0).heading.epsilonEquals(
                    Angle.deg(0)
            ) && wheelPoses.get(1).heading.epsilonEquals(Angle.deg(90))) {
                names = new String[]{"parallelEnc", "perpEnc"};
            } else names = new String[]{"enc0", "enc1"};
            forwardTicks = () -> mapTicks(trackingWheelLocalizer.getWheelPositions(),
                    wheelPoses, (p) -> p.heading.cos());
            lateralTicks = () -> mapTicks(trackingWheelLocalizer.getWheelPositions(),
                    wheelPoses, (p) -> p.heading.sin());
            encoders = Arrays.asList(
                    new EncoderData(names[0], wheelPoses.get(0).heading, () -> trackingWheelLocalizer.getWheelPositions().get(0)),
                    new EncoderData(names[1], wheelPoses.get(1).heading, () -> trackingWheelLocalizer.getWheelPositions().get(1))
            );
        } else {
            forwardTicks = null;
            lateralTicks = null;
            encoders = Collections.emptyList();
        }
    }

    private double mapTicks(
            List<Double> encPositions,
            List<Pose2d> wheelPoses,
            Function<Pose2d, Double> weightFunction
    ) {
        double ticks = 0;
        double norm = 0;
        for (int i = 0; i < 3; i++) {
            double mult = weightFunction.apply(wheelPoses.get(i));
            ticks += encPositions.get(i) * mult;
            norm += mult;
        }
        return ticks / norm;
    }
}