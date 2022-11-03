package org.firstinspires.ftc.teamcode;

import android.os.Build;
import androidx.annotation.RequiresApi;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.*;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledFuture;

@RequiresApi(api = Build.VERSION_CODES.N)
@TeleOp(name="FTC2023")
public class MainTeleOp extends OpMode {

    private Robot robot;
    private Controller controller1;

    private double speed_limit;
    private int k = 0;
    private double raise_percentage;
    public double RAISE_POWER = 1.0;
    private ScheduledFuture<?> lastSliderRaised;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, Executors.newScheduledThreadPool(1));
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
        if (controller1.leftBumper()) {
            speed_limit = 0.85;
        } else {
            speed_limit = 0.5;
        }
        double x = controller1.left_stick_x * speed_limit;
        double y = -controller1.left_stick_y * speed_limit;
        double r = -controller1.right_stick_x * speed_limit;
        robot.wheels.move(x, y, r);

        // TODO: apply a button to enable/disable headless moving
        // TODO: watch out for the differences between buttons dualshock 4

        // ------- printing the slider position --------
        robot.slider.getCurrentPositionSlider();

        // ------- controlling the gripper -------
        if (controller1.dpadRightOnce()) {
            robot.gripper.release();
        } else if (controller1.dpadLeftOnce()) {
            robot.gripper.grab();
        }

        // ------- controlling the slider on predefined positions -----
        if(!Utils.isDone(lastSliderRaised)) {
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
        lastSliderRaised = robot.slider.raise(raise_percentage, RAISE_POWER);

        // ------- printing out the position of the slider --------
        telemetry.addData("Slider position", raise_percentage);
    }

}