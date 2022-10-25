package org.firstinspires.ftc.teamcode;

import android.os.Build;
import androidx.annotation.RequiresApi;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.*;

import java.util.concurrent.Executors;

@RequiresApi(api = Build.VERSION_CODES.N)
@TeleOp(name="FTC2023")
public abstract class MainTeleOp extends OpMode {

    private Robot robot;
    private Controller controller1;

    private boolean turbo = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, Executors.newScheduledThreadPool(1));

        controller1 = new Controller(gamepad1);

        robot.gripper.release();
    }

    public void stop() { robot.stopRobot(); }

    @Override
    public void loop() {
        controller1.update();

        double x = controller1.left_stick_x;
        double y = controller1.left_stick_y;
        double r = controller1.right_stick_x;

        robot.wheels.move(x, y, r);

        if (controller1.AOnce()) {
             robot.gripper.grab();
        } else if (controller1.BOnce()) {
            robot.gripper.release();
        }

    }


}