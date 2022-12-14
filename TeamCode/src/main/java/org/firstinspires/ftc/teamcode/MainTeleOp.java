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
    private int raise_value;
    private boolean useArcadeMode; // TODO: add arcade / headless movement
    public double RAISE_POWER = 1.0;
    private ScheduledFuture<?> lastSliderRaised1;

    @Override
    public void init() {
        robot = new Robot(
                hardwareMap,
                telemetry,
                Executors.newScheduledThreadPool(1)
        );
        controller1 = new Controller(gamepad1);

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

        // TODO: apply a button to enable/disable headless moving

        // ------- controlling the gripper -------
        if (controller1.dpadUp()) {
            robot.gripper.grab();
        } else if (controller1.dpadDown()) {
            robot.gripper.release();
        }

        // ------- controlling the slider on predefined positions -----
        if (controller1.YOnce()) {
            raise_value = 8000;
        } else if (controller1.BOnce()) {
            raise_value = 5000;
        } else if (controller1.XOnce()) {
            raise_value = 3000;
        } else if (controller1.AOnce()) {
            raise_value = 0;
        } else {
            return ;
        }

        // --------- canceling the slider movement ----------
        if (controller1.rightBumper()) {
            lastSliderRaised1 = robot.slider.raiseSlider(0, RAISE_POWER);
        }

        // -------- controlling the slider using the triggers --------
        if (raise_value <= 0.9) {
            raise_value += controller1.right_trigger * 1000;
        } else if (raise_value >= 0.1) {
            raise_value -= controller1.left_trigger * 1000;
        }

        // ------- moving the slider -------
        if (!Utils.isDone(lastSliderRaised1)) {
            lastSliderRaised1 = robot.slider.raiseSlider(raise_value, RAISE_POWER);
        }

        // ------- printing the slider position --------
        // TODO: fix the telemetry printing
        telemetry.addData("Raise value target", raise_value);
        telemetry.addData("Slider position", robot.slider.getCurrentPositionSlider());

        telemetry.update();
    }

}