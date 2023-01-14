package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.util.Objects;

public class Imu {
    // ------ initializing the variables -------
    private double headingOffset = 0.0;
    private Orientation angles;
    private Acceleration gravity;

    private HardwareMap map;
    private Telemetry telemetry;
    public BNO055IMU imu_sensor;

    Imu(@NonNull final Parameters parameters) {
        telemetry = Objects.requireNonNull(parameters.telemetry, "Telemetry object was not set up");
        map = Objects.requireNonNull(parameters.map, "HardwareMap object was not set up");

        imu_sensor = map.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imu_parameters = new BNO055IMU.Parameters();
        imu_parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu_parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu_parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu_parameters.loggingEnabled = true;
        imu_parameters.loggingTag = "IMU";
        imu_parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu_sensor.initialize(imu_parameters);
    }

    public void loop() {
        angles = imu_sensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        gravity = imu_sensor.getGravity();
    }

    public double getHeading() {
        return (angles.firstAngle - headingOffset) % (2.0 * Math.PI);
    }

    public void resetHeading() {
        headingOffset = angles.firstAngle;
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(getHeading());
    }

    public static class Parameters {
        public Telemetry telemetry = null;
        public HardwareMap map = null;

    }

}