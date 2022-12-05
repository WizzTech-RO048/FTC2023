package org.firstinspires.ftc.teamcode.Autonomy;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class Detector {

    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public static final int LEFT = 1;
    public static final int MID = 2;
    public static final int RIGHT = 3;

    public AprilTagDetection detectPosition(ArrayList<AprilTagDetection> current_positions,
                                            Telemetry telemetry) {

        AprilTagDetection detected_position = null;

        boolean tag_found = false;

        for (AprilTagDetection tag : current_positions) {
            if (tag.id == LEFT || tag.id == RIGHT || tag.id == MID) {
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