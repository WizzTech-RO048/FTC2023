package org.firstinspires.ftc.teamcode.Autonomy.States;

import android.os.Build;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.ScheduledExecutorService;

import org.firstinspires.ftc.teamcode.Autonomy.Detector;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@RequiresApi(api = Build.VERSION_CODES.N)
class StateInitial extends State {
    private final Detector.Position mockPosition;

    StateInitial(HardwareMap hardwareMap,
                 Telemetry telemetry,
                 ScheduledExecutorService service,
                 @Nullable Detector.Position mockedPosition) {
        super(new Robot(hardwareMap, telemetry, service));
        mockPosition = mockedPosition;
    }

    @Override
    public State update() {
        return new StateBarcodeDetect(robot, mockPosition);
    }
}