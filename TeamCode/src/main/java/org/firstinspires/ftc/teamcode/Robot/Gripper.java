package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class Gripper {
    private final double RELEASE_POSITION = 0.5;
    private final double GRAB_POSITION = 1.0;

    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;

    private final Servo gripper;

    Gripper(@NonNull final Parameters parameters) {
        telemetry = Objects.requireNonNull(parameters.telemetry, "Telemetry object was not set");
        hardwareMap = Objects.requireNonNull(parameters.hardwareMap, "HardwareMap was not set");

        gripper = hardwareMap.get(Servo.class, "gripper");
    }

    public void grab() {
        gripper.setPosition(GRAB_POSITION);
    }

    public void release() {
        gripper.setPosition(RELEASE_POSITION);
    }

    public static class Parameters {
        public HardwareMap hardwareMap;
        public Telemetry telemetry;
    }
}
