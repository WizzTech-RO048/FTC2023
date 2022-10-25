package org.firstinspires.ftc.teamcode.Robot;

import android.os.Build;
import androidx.annotation.RequiresApi;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.concurrent.ScheduledExecutorService;


@RequiresApi(api = Build.VERSION_CODES.N)
public class Robot {

    public Telemetry telemetry;

    public Wheels wheels;
    public Gripper gripper;
    public Slider slider;

    public Robot(final HardwareMap hardwareMap, final Telemetry t, ScheduledExecutorService scheduler) {
        telemetry = t;

        // --------- initializing the imu sensor --------
        // BNO055IMU imu_sensor = hardwareMap.get(BNO055IMU.class, "imu_sensor");
        // imu_sensor.initialize(new BNO055IMU.Parameters());

        // ----- parsing the parameters for initializing the Wheels class ----
        Wheels.Parameters wheels_parameters = new Wheels.Parameters();
        wheels_parameters.hardwareMap = hardwareMap;
        wheels_parameters.telemetry = telemetry;
        // wheels_parameters.imu_sensor = imu_sensor;
        wheels_parameters.scheduler = scheduler;
        wheels_parameters.rpm = 435;
        wheels_parameters.encoder_resolution = 384.5;
        wheels = new Wheels(wheels_parameters);

        // ----- parsing the parameters for initializing the Gripper class -----
        Gripper.Parameters gripper_parameters = new Gripper.Parameters();
        gripper_parameters.telemetry = telemetry;
        gripper_parameters.hardwareMap = hardwareMap;
        gripper = new Gripper(gripper_parameters);

        // TODO: add the slider
        Slider.Parameters slider_parameters = new Slider.Parameters();
        slider_parameters.armRaisedPosition = 5200; // 5200 is the maximum
        slider_parameters.telemetry = telemetry;
        slider_parameters.hardwareMap = hardwareMap;
        slider_parameters.scheduler = scheduler;
        slider = new Slider(slider_parameters);
    }


    public void stopRobot() {
        wheels.stopEngines();
    }

}
