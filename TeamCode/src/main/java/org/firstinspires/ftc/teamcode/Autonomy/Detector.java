package org.firstinspires.ftc.teamcode.Autonomy;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class Detector {

    AprilTagDetectionPipeline aprilTagDetectionPipeline;

     public enum Position {
        LEFT(1),
        MIDDLE(2),
        RIGHT(3);

        private final int i;
        Position(int i) {
            this.i = i;
        }

        @Override
        public String toString() {
            switch (this) {
                case LEFT:
                    return "LEFT";
                case MIDDLE:
                    return "MIDDLE";
                case RIGHT:
                    return "RIGHT";
                default:
                    return "None";
            }
        }
    }

    public AprilTagDetection detectPosition(ArrayList<AprilTagDetection> current_position,
                                            Telemetry telemetry) {
        /**
         * This function determined on which position the cone was set.
         *
         * @param current_positions
         * @param telemetry Using the telemetry to output data
         */

        AprilTagDetection detected_position = null;

        boolean tag_found = false;

        for (AprilTagDetection tag : current_position) {
            if (tag.id == Position.LEFT || tag.id == Position.MIDDLE || tag.id == Position.RIGHT) {
                detected_position = tag;
                tag_found = true;
                break;
            }
        }

        if (tag_found) {
            telemetry.addLine("Tag detected");
        } else {
            telemetry.addLine("Any tags detected");
            if (detected_position == null) {
                telemetry.addLine("The tag was never seen");
            } else {
                telemetry.addLine("The tag was seen");
            }
        }

        telemetry.update();

        return detected_position;
    }

}