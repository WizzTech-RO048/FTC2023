package org.firstinspires.ftc.teamcode.Autonomy.States.Scenarios;

import android.os.Build;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;
import org.firstinspires.ftc.teamcode.Autonomy.Detector;
import org.firstinspires.ftc.teamcode.Autonomy.States.*;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Wheels;

@RequiresApi(api = Build.VERSION_CODES.N)
public class StatePositionLeft extends State {
    private boolean hasMovedForward = false;
    private boolean hasMovedLeft = false;

    public StatePositionLeft(@NonNull Robot robot) {
        super(robot);
    }

    @Override
    public State update() {

        if (!hasMovedForward) {
            hasMovedForward = true;
            return new StateMove(robot, 0.5, 0.3, Wheels.MoveDirection.FORWARD, this, 10);
        }

        if (!hasMovedLeft) {
            hasMovedLeft = true;
            return new StateMove(robot, 0.5, 0.3, Wheels.MoveDirection.LEFT, this, 10);
        }

        return this;
    }
}
