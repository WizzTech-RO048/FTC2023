package org.firstinspires.ftc.teamcode.Robot;


import android.graphics.drawable.GradientDrawable;
import android.os.Build;
import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import kotlin.text.CharDirectionality;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.ScheduledExecutorService;
import java.util.stream.Stream;


// TODO: add the new robot movement code
@RequiresApi(api = Build.VERSION_CODES.N)
public class Wheels {
    private static final List<String> MOTORS_NAMES = Arrays.asList(
            "left_front",
            "left_rear",
            "right_front",
            "right_rear"
    );

    private List<DcMotorEx> engines = new ArrayList<>();
    public DcMotor lf, lr, rf, rr;

    private final Telemetry telemetry;
    private final ScheduledExecutorService scheduler;
    private final HardwareMap map;

    // ------- imu variables ---------
    private final Imu imu_sensor;

    Wheels(@NonNull final Parameters parameters) {
        telemetry = Objects.requireNonNull(parameters.telemetry, "Telemetry object was not set");
        scheduler = Objects.requireNonNull(parameters.scheduler, "Scheduler was not set");
        map = Objects.requireNonNull(parameters.hardwareMap, "HardwareMap was not passed");

        Imu.Parameters imu_parameters = new Imu.Parameters();
        imu_parameters.map = map;
        imu_parameters.telemetry = telemetry;
        imu_sensor = new Imu(imu_parameters);
        imu_sensor.loop();

        // --------- setting up the engines individually -----
        lf = setEngine(map, true, true, "lf");
        lr = setEngine(map, true, true,"lr");
        rf = setEngine(map, true, true, "rf");
        rr = setEngine(map, true, true, "rr");

    }

    private static DcMotor setEngine(HardwareMap map,
                                       boolean useZeroPowerBehaviorBraking,
                                       boolean useEncoders,
                                       String name) {
        /*
         * Initializing an individual engine by hardware mapping the name, setting the direction, the
         * running mode, the condition of using encoders and enabling ZeroPowerBehavior
         *   @param map HardwareMap for reading the engine from the driver station configuration
         *   @param name The name of the motor in the driver station configuration
         *   @param useEncoder The condition if we want to use the encoder for the engine
         *   @param useZeroPowerBehaviorBraking The condition if we want to use the brakes
         *
         *   @returns motor: The engine, as a DcMotorEx type of variable
         * */

        // -------- reading the engine from the HardwareMap --------
        DcMotor engine = map.get(DcMotor.class, name);

        // --------- setting the direction of the engine ---------
        engine.setDirection(name.contains("left") ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);

        // ------- setting the encoders for the engine ------
        engine.setMode(useEncoders ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // -------- enabling the usage of braking when the engine is idle -------
        engine.setZeroPowerBehavior(useZeroPowerBehaviorBraking ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);

        return engine;
    }

    public void setMotors(double x, double y, double rotation, boolean useArcadeMode) {
        x = Math.pow(x, 3.0);
        y = Math.pow(y, 3.0);
        rotation = Math.pow(rotation, 3.0);

        final double direction = Math.atan2(x, y) + (useArcadeMode ? 0.0 : imu_sensor.getHeading());
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        final double lf = speed * Math.sin(direction + Math.PI / 4.0) + rotation;
        final double rf = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
        final double lr = speed * Math.cos(direction + Math.PI / 4.0) + rotation;
        final double rr = speed * Math.sin(direction + Math.PI / 4.0) - rotation;

        setMotors(lf, lr, rf, rr);
    }

    public void setMotors(double _lf, double _lr, double _rf, double _rr) {
        final double scale = maxAbs(1.0, _lf, _lr, _rf, _rr);
        lf.setPower(_lf / scale);
        lr.setPower(_lr / scale);
        rf.setPower(_rf / scale);
        rr.setPower(_rr / scale);
    }

    private static double maxAbs(double... xs) {
        double ret = Double.MIN_VALUE;
        for (double x : xs) {
            if (Math.abs(x) > ret) {
                ret = Math.abs(x);
            }
        }
        return ret;
    }

    public void stopEngines() {
        for (DcMotorEx engine : engines) {
            engine.setPower(0.0);
        }
    }

    public static class Parameters {
        public HardwareMap hardwareMap = null;
        public Telemetry telemetry = null;

        public ScheduledExecutorService scheduler = null;
        public boolean use_encoders = true;
        public boolean useZeroPowerBehaviorBraking = true;

        public double rpm = 0;
    }
}