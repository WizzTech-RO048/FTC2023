package org.firstinspires.ftc.teamcode.Autonomy.States;

import android.os.Build;
import androidx.annotation.NonNull;

import androidx.annotation.RequiresApi;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Autonomy.States.State;

@RequiresApi(api = Build.VERSION_CODES.N)
public
class StateException extends State {
    public StateException(@NonNull Robot robot,
                          Exception e) {
        super(robot);
        robot.getTelemetry().addData("Unhandled exception", e);
    }

    @Override
    public State update() {
        return this;
    }
}