package org.firstinspires.ftc.teamcode.Autonomy.States;

import android.os.Build;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Robot.*;
import org.firstinspires.ftc.teamcode.Autonomy.*;

import java.util.concurrent.Executors;

@RequiresApi(api = Build.VERSION_CODES.N)
public abstract class State {
    protected final Robot robot;

    /** The current time, in seconds. */
    protected double time = 0;

    /** The time this state was initialized, in seconds. */
    protected double startTime = 0;

    /** Seconds passed since this state was initialized. */
    protected double timePassed = 0;

    protected State(@NonNull Robot robot) {
        this.robot = robot;
    }

    public void setTime(double time) {
        this.time = time;
        if (startTime == 0) {
            startTime = time;
        }
        timePassed = time - startTime;
    }

    public abstract State update();

    public void stop() {
        robot.stopRobot();

        // TODO: stop the camera
//        robot.camera.stop();
    }

    public static State initial(HardwareMap hardwareMap, Telemetry telemetry, @Nullable Detector.Position mockPosition) {
        return new StateInitial(hardwareMap, telemetry, Executors.newScheduledThreadPool(1), mockPosition);
    }

    // FIXME: doesn't work
    public void waitForSeconds(double seconds){
        int sec = (int) (seconds * 1000);
        try{
            Thread.sleep(sec);
        } catch (InterruptedException e){
            e.printStackTrace();
        }
    }
}