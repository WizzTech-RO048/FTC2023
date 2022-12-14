package org.firstinspires.ftc.teamcode.Autonomy.States;

import android.os.Build;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;
import org.firstinspires.ftc.teamcode.Autonomy.Detector;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Autonomy.States.Scenarios.*;

@RequiresApi(api = Build.VERSION_CODES.N)
class StateBarcodeDetect extends State {
    private final Detector.Position mockPosition;

    StateBarcodeDetect(@NonNull Robot robot,
                       @Nullable Detector.Position mockedPosition) {
        super(robot);
        mockPosition = mockedPosition;
    }

    @Override
    public State update() {
        if (mockPosition != null) {
            return nextState(mockPosition);
        }

        return new StateException(robot, new Exception("Failed to load"));
    }

    private State nextState(Detector.Position position) {
        switch (position) {
            case LEFT:
                return new StatePositionLeft(robot);
            case MIDDLE:
                return new StatePositionMiddle(robot);
            case RIGHT:
                return new StatePositionRight(robot);
            default:
                return new StateException(robot, new RuntimeException("No barcode position detected"));
        }
    }

}