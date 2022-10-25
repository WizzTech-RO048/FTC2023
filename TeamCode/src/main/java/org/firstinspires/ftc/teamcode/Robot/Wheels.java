package org.firstinspires.ftc.teamcode.Robot;


import android.os.Build;
import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.ScheduledExecutorService;


@RequiresApi(api = Build.VERSION_CODES.N)
public class Wheels {
    private static final List<String> MOTORS_NAMES = Arrays.asList(
            "left_front",
            "left_rear",
            "right_front",
            "right_rear"
    );

    private List<DcMotorEx> engines = new ArrayList<>();
    private final double encoderTicksPerSecond;


    // private final BNO055IMU imu_sensor;
    private final Telemetry telemetry;
    private final ScheduledExecutorService scheduler;
    private final HardwareMap map;

    private static DcMotorEx getEngine(HardwareMap map, String name) {
        DcMotorEx motor = map.get(DcMotorEx.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return motor;
    }

    Wheels(@NonNull final Parameters parameters) {
        telemetry = Objects.requireNonNull(parameters.telemetry, "Telemetry object was not set");
        // imu_sensor = Objects.requireNonNull(parameters.imu_sensor, "IMU sensor was not set");
        scheduler = Objects.requireNonNull(parameters.scheduler, "Scheduler was not set");
        map = Objects.requireNonNull(parameters.hardwareMap, "HardwareMap was not passed");


        for (String motor_name : MOTORS_NAMES) {
            engines.add(getEngine(map, motor_name));
        }

        if (parameters.encoder_resolution != 0 && parameters.rpm != 0) {
            encoderTicksPerSecond = (parameters.rpm / 60) * parameters.encoder_resolution;
            useEncoders(true);
        } else {
            encoderTicksPerSecond = 0;
            useEncoders(false);
        }

        useBrakes(true);
    }

    private void useEncoders(boolean condition) {
        DcMotor.RunMode mode = condition ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        for(DcMotor engine : engines) {
            engine.setMode(mode);
        }
    }

    private void useBrakes(boolean condition) {
        DcMotor.ZeroPowerBehavior behavior = condition ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        for(DcMotor engine : engines) {
            engine.setZeroPowerBehavior(behavior);
        }
    }

    // TODO: check why we need this specific version of the API
    public void move(double x, double y, double r) {
        // ------ normalizing the values from the joysticks --------
        x = Math.max(-1, Math.min(x, 1));
        y = Math.max(-1, Math.min(y, 1));
        r = Math.max(-1, Math.min(r, 1));

        double[] input = {
                x + y + r, // left front
                y - x + r, // left rear
                y - x - r, // left rear
                y + x - r, // right rear
        };

        double highest = Arrays.stream(input).map(Math::abs).reduce(1, Math::max);

        // -------- giving motors the power for the movement ------
        for(int i = 0; i < input.length; i++) {
            setPower(engines.get(i), input[i] / highest);
        }
    }

    private void setPower(DcMotorEx engine, double power) {
        /*
         * Setting up the power in the engines
         *  @param engine The engine that we want to use
         *  @param power A value between [0.0, 1.0] that represents the power that we want the engine to move
         * */
        // TODO: what happens if the power parameter is a negative value

        if (engine.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            engine.setVelocity(Math.round(power * encoderTicksPerSecond));
        } else {
            engine.setPower(power);
        }
    }

    public void stopEngines() {
        engines.forEach(engine -> setPower(engine, 0.0));
    }


    public static class Parameters {
        public HardwareMap hardwareMap = null;
        public Telemetry telemetry = null;
        // public BNO055IMU imu_sensor = null;
        public ScheduledExecutorService scheduler = null;
        public double encoder_resolution = 0.0;
        public double rpm = 0;
    }
}