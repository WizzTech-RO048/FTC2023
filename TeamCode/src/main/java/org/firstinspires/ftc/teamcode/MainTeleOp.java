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

    private boolean turbo = false;
    private double slider_position;
    public double RAISE_POWER = 0.5;
    private ScheduledFuture<?> lastSliderRaised;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, Executors.newScheduledThreadPool(1));
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
        double x = controller1.left_stick_x;
        double y = controller1.left_stick_y;
        double r = controller1.right_stick_x;
        robot.wheels.move(x, y, r);

        // ------- printing the slider position --------
        robot.slider.getCurrentPositionSlider();

        // ------- controlling the gripper -------
        if (controller1.rightBumper()) {
            robot.gripper.grab();
        } else if (controller1.leftBumper()) {
            robot.gripper.release();
        }

        // ------- controlling the slider --------
        if(!Utils.isDone(lastSliderRaised)) {
            return ;
        }
        if (controller1.AOnce()) {
            slider_position = 0.3;
        } else if (controller1.BOnce()) {
            slider_position = 0.6;
        } else if (controller1.YOnce()) {
            slider_position = 0.9;
        } else if (controller1.XOnce()) {
            slider_position = 0.0;
        } else {
            return ;
        }
        lastSliderRaised = robot.slider.raiseArm(slider_position, RAISE_POWER);
    }

}