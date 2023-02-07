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

    private int raise_limit = 3200, raise_value;
    private int rightLimit = 3000, leftLimit = 3000;
    public double RAISE_POWER = 1.0;
    private Pair<ScheduledFuture<?>, ScheduledFuture<?>> lastMove;

    private boolean gripperOpen = true;

    @Override
    public void init() {
        robot = new Robot(
                hardwareMap,
                telemetry,
                Executors.newScheduledThreadPool(1)
        );
        controller1 = new Controller(gamepad1);

        // --------- initializing the robot --------
        gripperOpen = robot.gripper.release();

    }

    // ------ the emergency stop function ---------
    public void stop() { robot.stopRobot(); }

    @Override
    public void loop() {
        controller1.update();

        // ------- printing the slider position --------
        telemetry.addData("Raise target", raise_value);
        telemetry.addData("Right slider position", robot.slider.getCurrentPositionSlider("right"));
        telemetry.addData("left slider position", robot.slider.getCurrentPositionSlider("left"));
        telemetry.addData("Gripper open", gripperOpen);
        telemetry.update();

        // -------- controlling the robot movement ------
        double x = controller1.left_stick_x;
        double y = controller1.left_stick_y;
        double r = -controller1.right_stick_x;

        robot.wheels.move(x, y, r, false);

        // ------- controlling the gripper -------
        if (controller1.dpadUp()) {
            gripperOpen = robot.gripper.release();
        } else if (controller1.dpadDown()) {
            gripperOpen = robot.gripper.grab();
        }

        ScheduledFuture<?> rightMove = null, leftMove = null;

        // ------- controlling the slider positions -----
        if(!Utils.isDone(leftMove) || !Utils.isDone(rightMove)) { return ; }
        else if (controller1.YOnce() && gripperOpen) { // 100%
            raise_value = raise_limit;
            leftMove = robot.slider.raiseLeftSlider(raise_value, RAISE_POWER);
            rightMove = robot.slider.raiseRightSlider(raise_value, RAISE_POWER);
        } else if (controller1.BOnce() && gripperOpen) { // 75%
            raise_value = (int)((raise_limit * 75) / 100);
            leftMove = robot.slider.raiseLeftSlider(raise_value, RAISE_POWER);
            rightMove = robot.slider.raiseRightSlider(raise_value, RAISE_POWER);
        } else if (controller1.XOnce() && gripperOpen) { // 15%
            raise_value = (int)((raise_limit * 15) / 100);
            leftMove = robot.slider.raiseLeftSlider(raise_value, RAISE_POWER);
            rightMove = robot.slider.raiseRightSlider(raise_value, RAISE_POWER);
        } else if (controller1.AOnce()) { // 0%
            raise_value = 0;
            gripperOpen = robot.gripper.release();
            leftMove = robot.slider.raiseLeftSlider(0, RAISE_POWER);
            rightMove = robot.slider.raiseRightSlider(0, RAISE_POWER);
        }
        // ------ controlling the slider raising using the triggers -----
        else if (raise_value <= raise_limit && controller1.right_trigger != 0.0) {
            raise_value = (int) (raise_value + controller1.right_trigger * 100);
            leftMove = robot.slider.raiseLeftSlider(raise_value, RAISE_POWER);
            rightMove = robot.slider.raiseRightSlider(raise_value, RAISE_POWER);
        } else if (raise_value >= 0 && controller1.left_trigger != 0.0) {
            raise_value = (int) (raise_value - controller1.left_trigger * 100);
            leftMove = robot.slider.raiseLeftSlider(raise_value, RAISE_POWER);
            rightMove = robot.slider.raiseRightSlider(raise_value, RAISE_POWER);
        } else { return ; }




        telemetry.update();
    }

}