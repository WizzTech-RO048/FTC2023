package org.firstinspires.ftc.teamcode.Robot;


import android.os.Build;
import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import kotlin.Triple;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.*;
import java.util.concurrent.ScheduledExecutorService;


@RequiresApi(api = Build.VERSION_CODES.N)
public class Wheels {
    // ---------- engines variables for setting up each engine -------
    private List<DcMotorEx> engines = new ArrayList<>();
    public DcMotor lf, lr, rf, rr;
    private static final List<String> MOTORS_NAMES = Arrays.asList(
            "left_front",
            "left_rear",
            "right_front",
            "right_rear"
    );

    // -------- odometry vairables ----------
    // public DcMotor encoderLeft, encoderRight, encoderAux;
    // final static double L = 5.1; // distance between encoder 1 and 2 in cm
    // final static double B = 15.7; // distance between midpoint of encoder 1 & 2 and encoder 3
    // final static double R = 3.0; // wheel radius in cm
    // final static double N = 8192; // encoder ticks per revolution; REV encoder
    // final static double cm_per_tick = 2.0 * Math.PI, * R / N;

    // -------- variables for keeping track of encoders position
    private int currentLeftPosition = 0;
    private int currentRightPosition = 0;
    private int currentAuxPosition = 0;

    private int previousLeftPosition = 0;
    private int previousRightPosition = 0;
    private int previousAuxPosition = 0;

    // ------- 'slow down' before sudden changes of direction variables ---------
    private int VMAX = 100;
    public Triple<Double, Double, Double>[] joystickReadings = new Triple[VMAX];
    private int joystickReadingSize = 3;
    private double slowDownPercentage = 1;

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
        lf = setEngine(map, true, true, "left_front");
        lr = setEngine(map, true, true,"left_rear");
        rf = setEngine(map, true, true, "right_front");
        rr = setEngine(map, true, true, "right_rear");

        // encoderLeft = lr;
        // encoderRight = rr;
        // encoderAux = rf;

//        resetDriverEncoders();
    }

    private void resetDriverEncoders() {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // public XyhVector START_POS = new XyhVector(213, 102, Math.toRadians(-174));
    // public XyhVector pos = new XyhVector(START_POS);
    //
    //
    // // TODO: write this class as it should be written / search in on the web
    // public static class XyhVector {
    //     public int x;
    //     public int y;
    //     public int h;
    // }
    //
    // public void odometry() {
    //     previousLeftPosition = currentLeftPosition;
    //     previousRightPosition = currentRightPosition;
    //     previousAuxPosition = currentAuxPosition;
    //
    //     currentLeftPosition = -encoderRight.getCurrentPosition();
    //     currentLeftPosition = -encoderLeft.getCurrentPosition();
    //     currentAuxPosition = encoderAux.getCurrentPosition();
    //
    //     int dn1 = currentLeftPosition - previousLeftPosition;
    //     int dn2 = currentRightPosition - previousRightPosition;
    //     int dn3 = currentAuxPosition - previousAuxPosition;
    //
    //     double dtheta = cm_per_tick * (dn2-dn1) / L;
    //     double dx = cm_per_tick * (dn1+dn2) / 2.0;
    //     double dy = cm_per_tick * (dn3 - (dn2-dn1) * B / L);
    //
    //     double theta = pos.h + (dtheta / 2.0);
    //     pos.x += dx * Math.cos(theta) - dy * Math.sin(theta);
    //     pos.y += dx + Math.sin(theta) + dy * Math.cos(theta);
    //     pos.h += dtheta;
    // }

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
        // WARNING: this might not work
//        engine.setDirection(name.contains("left") || name.contains("right") ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        // -------------- THE FIX --------------
         if (name.contains("left_front")) {
             engine.setDirection(DcMotorSimple.Direction.FORWARD);
         } else if (name.contains("left_rear")) {
             engine.setDirection(DcMotorSimple.Direction.FORWARD);
         } else if (name.contains("right_front")) {
             engine.setDirection(DcMotorSimple.Direction.REVERSE);
         } else if (name.contains("right_rear")) {
             engine.setDirection(DcMotorSimple.Direction.REVERSE);
         }

        // ------- setting the encoders for the engine ------
        engine.setMode(useEncoders ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // -------- enabling the usage of braking when the engine is idle -------
        engine.setZeroPowerBehavior(useZeroPowerBehaviorBraking ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);

        return engine;
    }


    private double calculateSlowDownPercentage(Triple<Double, Double, Double>[] joystickReadings) {
        /**
         * The goal of this function is to increase the robot
         * */
        double percentage = 1.0;

        double x_axis_diff = joystickReadings[2].getFirst() - joystickReadings[1].getFirst();
        double y_axis_diff = joystickReadings[2].getSecond() - joystickReadings[1].getSecond();
        double r_axis_diff = joystickReadings[2].getThird() - joystickReadings[1].getThird();

        if(x_axis_diff > 1.0 || y_axis_diff > 1.0 || r_axis_diff > 1.0) { percentage = 0.5; }
        if (x_axis_diff < 0.2 || y_axis_diff < 0.2 || r_axis_diff < 0.2) { percentage = 1.0; }

        return percentage;
    }

    public void move(double x, double y, double rotation, boolean useArcadeMode) {

        // TODO: think about the implementation of this 'slow down' feature
        // --------- slowing down between sudden changes of direction ----------
        // Triple<Double, Double, Double> coordinates = new Triple<>(x, y, rotation);
        // if (values_read >= joystickReadingSize) {
        //     for(int i = 1; i < joystickReadingSize; i++) {
        //         joystickReadings[i-1] = joystickReadings[i];
        //     }
        //     joystickReadings[joystickReadingSize-1] = coordinates;
        // } else {
        //     joystickReadings[values_read] = coordinates;
        //     values_read++;
        // }
        //
        // if (joystickReadings.length == joystickReadingSize) {
        //     slowDownPercentage = calculateSlowDownPercentage(joystickReadings);
        // } else {
        //     slowDownPercentage = 1;
        // }

        x = Math.pow(x * slowDownPercentage, 3.0);
        y = Math.pow(y * slowDownPercentage, 3.0);
        rotation = Math.pow(rotation, 3.0);

        final double direction = Math.atan2(x, y) + (useArcadeMode ? 0.0 : imu_sensor.getHeading());
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        final double lf = speed * Math.cos(direction + Math.PI / 4.0) + rotation;
        final double rf = speed * Math.sin(direction + Math.PI / 4.0) - rotation;
        final double lr = speed * Math.sin(direction + Math.PI / 4.0) + rotation;
        final double rr = speed * Math.cos(direction + Math.PI / 4.0) - rotation;

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