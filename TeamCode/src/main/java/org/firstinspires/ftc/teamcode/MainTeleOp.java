package org.firstinspires.ftc.teamcode;

import android.os.Build;
import android.util.Pair;
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
    private int raise_value;
    public double RAISE_POWER = 1.0;
    private ScheduledFuture<?> lastRightMove, lastLeftMove;

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
        double x = controller1.left_stick_x;
        double y = controller1.left_stick_y;
        double r = -controller1.right_stick_x;

        robot.wheels.move(y, x, r, true);

        // ------- controlling the gripper -------
        if (controller1.dpadUp()) {
            robot.gripper.release();
            // TODO: implement double grabbing
        } else if (controller1.dpadDown()) {
            robot.gripper.grab();
        }

        // ------- controlling the slider positions -----
        if(!Utils.isDone(lastLeftMove) || !Utils.isDone(lastRightMove)) { return ; }
        else if (controller1.YOnce()) { raise_value = 4200; }
        else if (controller1.BOnce()) { raise_value = 3000; }
        else if (controller1.XOnce()) { raise_value = 1400; }
        else if (controller1.AOnce()) { raise_value = 0; }
        else if (raise_value <= 4000 && controller1.right_trigger != 0.0) {
            raise_value = (int) (raise_value + controller1.right_trigger * 1000);
        } else if (raise_value >= 0 && controller1.left_trigger != 0.0) {
            raise_value = (int) (raise_value - controller1.left_trigger * 1000);
        } else { return ; }

        // --------- canceling the slider movement ----------
        if (controller1.rightBumper()) {
            lastLeftMove = robot.slider.raiseLeftSlider(raise_value, RAISE_POWER);
            lastRightMove = robot.slider.raiseRightSlider(raise_value, RAISE_POWER);
        }

        // ------- moving the sliders -------
        lastLeftMove = robot.slider.raiseLeftSlider(raise_value, RAISE_POWER);
        lastRightMove = robot.slider.raiseRightSlider(raise_value, RAISE_POWER);

        // ------- printing the slider position --------
        // TODO: fix the telemetry printing
        telemetry.addData("Raise value target", raise_value);
        telemetry.addData("Slider position", robot.slider.getCurrentPositionSlider());

        telemetry.update();
    }

}