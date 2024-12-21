package org.firstinspires.ftc.teamcode.IntoDeep_SM.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCVCameraTest extends OpenCvPipeline {

    Telemetry telemetry;
    private Location location;
    public enum Location {
        LEFT,
        MIDDLE,
        RIGHT,
        NOT_FOUND
    }
    Mat mat = new Mat();
    static final Rect leftRect = new Rect(
            new Point(60,35),
            new Point(120,75)
    );
    static final Rect rightRect = new Rect(
            new Point(140,35),
            new Point(200,75)
    );



    static double PERCENT_COLOR_THRESHOLD = 0.4;

    public OpenCVCameraTest(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat,Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(241,92,12);
        Scalar highHSV = new Scalar(245,100,4);

        Core.inRange(mat,lowHSV,highHSV,mat);

        Mat left = mat.submat(leftRect);
        Mat right = mat.submat(rightRect);

        double leftValue = Core.sumElems(left).val[0] / leftRect.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / rightRect.area() / 255;

        left.release();
        right.release();

        boolean leftPosition = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean middlePosition = rightValue > PERCENT_COLOR_THRESHOLD;

        if (leftPosition && middlePosition) {
            location = Location.NOT_FOUND;
            telemetry.addData("Location", "not found");
        }
        else if (leftPosition) {
            location = Location.RIGHT;
            telemetry.addData("Location", "right");
        }
        else {
            location = Location.LEFT;
            telemetry.addData("Location", "left");
        }
        telemetry.update();

        return mat;
    }
    public Location getLocation() {
        return location;
    }
}
