package org.firstinspires.ftc.teamcode;

import android.os.Build;
import androidx.annotation.RequiresApi;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.*;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledFuture;

// TODO: configure the robot controller for the driver
@RequiresApi(api = Build.VERSION_CODES.N)
@TeleOp(name="FTC2023")
public class MainTeleOp extends OpMode {

    private Robot robot;
    private Controller controller1;

    private double speed_limit;
    private int k = 0;
    private double raise_percentage;
    private boolean useArcadeMode; // TODO: add arcade / headless movement
    public double RAISE_POWER = 1.0;
    private ScheduledFuture<?> lastSliderRaised1, lastSliderRaised2;

    @Override
    public void init() {
        robot = new Robot(
                hardwareMap,
                telemetry,
                Executors.newScheduledThreadPool(1)
        );
        controller1 = new Controller(gamepad1);

        // ---------- raising the slider a little bit ------------
//        lastSliderRaised = robot.slider.raise(Slider.Position.BASE, RAISE_POWER);

        // --------- initializing the robot --------
        robot.gripper.release();

    }

    // ------ the emergency stop function ---------
    public void stop() { robot.stopRobot(); }

    @Override
    public void loop() {
        controller1.update();

        // -------- controlling the robot movement ------
        double x = -controller1.left_stick_x;
        double y = controller1.left_stick_y;
        double r = -controller1.right_stick_x;

        robot.wheels.setMotors(y, x, r, true);

        // ------- printing the slider position --------
        // TODO: fix the telemetry printing
        robot.slider.getCurrentPositionSlider();
        telemetry.addData("Raise percentage", raise_percentage);

        robot.slider.getCurrentPositionSlider();

        // TODO: apply a button to enable/disable headless moving

        // ------- controlling the gripper -------
        if (controller1.dpadRightOnce()) {
            robot.gripper.grab();
        } else if (controller1.dpadLeftOnce()) {
            robot.gripper.release();
        }

        // ------- controlling the slider on predefined positions -----
        if(!Utils.isDone(lastSliderRaised1) || !Utils.isDone(lastSliderRaised2)) {
            return ;
        }
        if (controller1.YOnce()) {
            raise_percentage = 1.0;
        } else if (controller1.BOnce()) {
            raise_percentage = 0.7;
        } else if (controller1.XOnce()) {
            raise_percentage = 0.02;
        } else if (controller1.AOnce()) {
            raise_percentage = 0.0;
        } else {
            return ;
        }

        // -------- controlling the arm --------
        if (raise_percentage <= 0.9) {
            raise_percentage += (double) (controller1.right_trigger / 10);
        } else if (raise_percentage >= 0.1) {
            raise_percentage -= (double) (controller1.left_trigger / 10);
        }

        telemetry.addData("raise_percentage", raise_percentage);

        lastSliderRaised1 = robot.slider.raiseSlider(raise_percentage, RAISE_POWER);

        // ------- printing out the position of the slider --------
        telemetry.addData("Slider position", raise_percentage);

        telemetry.update();
    }

}