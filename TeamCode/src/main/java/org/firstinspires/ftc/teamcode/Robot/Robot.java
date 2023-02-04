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
    public Imu imu;

    public Robot(final HardwareMap hardwareMap, final Telemetry t, ScheduledExecutorService scheduler) {
        telemetry = t;

        // --------- initializing the imu sensor --------
        // BNO055IMU imu_sensor = hardwareMap.get(BNO055IMU.class, "imu_sensor");
        // imu_sensor.initialize(new BNO055IMU.Parameters());
        Imu.Parameters imu_parameters = new Imu.Parameters();
        imu_parameters.map = hardwareMap;
        imu_parameters.telemetry = telemetry;
        imu = new Imu(imu_parameters);

        // ----- parsing the parameters for initializing the Wheels class ----
        Wheels.Parameters wheels_parameters = new Wheels.Parameters();
        wheels_parameters.hardwareMap = hardwareMap;
        wheels_parameters.telemetry = telemetry;
        wheels_parameters.scheduler = scheduler;
        wheels_parameters.rpm = 435;
        wheels = new Wheels(wheels_parameters);

        // ----- parsing the parameters for initializing the Gripper class -----
        Gripper.Parameters gripper_parameters = new Gripper.Parameters();
        gripper_parameters.telemetry = telemetry;
        gripper_parameters.hardwareMap = hardwareMap;
        gripper = new Gripper(gripper_parameters);

        Slider.Parameters slider_parameters = new Slider.Parameters();
        slider_parameters.leftSliderLimit = 2500; // 5200 is the maximum
        slider_parameters.rightSliderLimit = 3000;
        slider_parameters.telemetry = telemetry;
        slider_parameters.hardwareMap = hardwareMap;
        slider_parameters.scheduler = scheduler;
        slider = new Slider(slider_parameters);
    }


    public void stopRobot() {
        wheels.stopEngines();
    }

}
