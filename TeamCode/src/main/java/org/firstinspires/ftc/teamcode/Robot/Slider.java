package org.firstinspires.ftc.teamcode.Robot;

import android.os.Build;
import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Utils;

import java.util.Objects;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

@RequiresApi(api = Build.VERSION_CODES.N)
public class Slider {

    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private final ScheduledExecutorService scheduler;

    private final DcMotorEx left_slider, right_slider;

    private final int armRaisedPosition;

    Slider(@NonNull final Parameters parameters) {
        scheduler = Objects.requireNonNull(parameters.scheduler, "Scheduler was not set");
        telemetry = Objects.requireNonNull(parameters.telemetry, "Telemetry was not set up");
        hardwareMap = Objects.requireNonNull(parameters.hardwareMap, "HardwareMap was not set up");

        left_slider = hardwareMap.get(DcMotorEx.class, "left_slider");
        left_slider.setDirection(DcMotorSimple.Direction.REVERSE);
        left_slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right_slider = hardwareMap.get(DcMotorEx.class, "right_slider");
        right_slider.setDirection(DcMotorSimple.Direction.REVERSE);
        right_slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armRaisedPosition = parameters.armRaisedPosition;
    }

    private ScheduledFuture<?> lastMove = null;

    public ScheduledFuture<?> raiseSlider(int targetPositionValue, double raisePower, String position) {
        if (!Utils.isDone(lastMove) && !lastMove.cancel(true)) {
            return null;
        }

//        int targetPosition = (int) Math.floor(Utils.interpolate(0, armRaisedPosition, positionPercentage, 1));
        int initialPosition = left_slider.getCurrentPosition();

        if (targetPositionValue == initialPosition) {
            return null;
        }

        if(position.equals("left")) {
            left_slider.setTargetPosition(targetPositionValue);
            left_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_slider.setPower(targetPositionValue > initialPosition ? raisePower : -raisePower);

            lastMove = Utils.poll(scheduler, () -> !left_slider.isBusy(), () -> left_slider.setPower(0), 10, TimeUnit.MILLISECONDS);
        } else {
            right_slider.setTargetPosition(targetPositionValue);
            right_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_slider.setPower(targetPositionValue > initialPosition ? raisePower : -raisePower);

            lastMove = Utils.poll(scheduler, () -> !left_slider.isBusy(), () -> right_slider.setPower(0), 10, TimeUnit.MILLISECONDS);
        }

        return lastMove;
    }

    public int getCurrentPositionSlider() {
        int slider_position = left_slider.getCurrentPosition();
        return slider_position;
    }

    public void stopSlider() {
        // ----- stopping the slider moving -----
//        lastMove.cancel(true);
        left_slider.setPower(0.0);
        right_slider.setPower(0.0);
    }

    public static class Parameters {
        public HardwareMap hardwareMap;
        public Telemetry telemetry;
        public ScheduledExecutorService scheduler;
        public int armRaisedPosition;
    }

}
