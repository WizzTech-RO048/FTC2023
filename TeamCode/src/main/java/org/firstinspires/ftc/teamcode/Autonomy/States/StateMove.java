package org.firstinspires.ftc.teamcode.Autonomy.States;

import android.os.Build;
import androidx.annotation.NonNull;

import java.util.concurrent.ScheduledFuture;

import androidx.annotation.RequiresApi;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Wheels;


@RequiresApi(api = Build.VERSION_CODES.N)
public class StateMove extends State {
    private final ScheduledFuture<?> movement;
    private final State previousState;
    private final double timeout;

    public StateMove(@NonNull Robot robot,
                     double meters,
                     double power,
                     Wheels.MoveDirection direction,
                     State previous,
                     double timeout) {
        super(robot);

        movement = robot.wheels.moveFor(meters, power, direction);
        previousState = previous;
        this.timeout = timeout;
    }

    @Override
    public State update() {
        if (movement.isDone()) {
            return previousState;
        }

        robot.getTelemetry().addData("Time", timePassed);
        robot.getTelemetry().update();

        if (timeout - timePassed < 0.5 && timeout != 0) {
            return previousState;
        }

        return this;
    }

    @Override
    public void stop() {
        movement.cancel(true);
        super.stop();
    }
}