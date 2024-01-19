package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.testing.sanke.CenterStageCVDetection;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class OpenCVDetector extends OpenCvPipeline {

    public static boolean DETECT_RED = false;
    public static double MINIMUM_VALUES = 100;
    public static double MAXIMUM_VALUES = 255;
    public static double MINIMUM_BLUE_RED = 0;
    public static double MAXIMUM_BLUE_RED = 100;
    public static Scalar RED = new Scalar(255, 0, 0);
    public static Scalar BLUE = new Scalar(0, 0, 255);

    Telemetry telemetry;
    Mat mat = new Mat();

    public enum Location {
        LEFT("Left"),
        RIGHT("Right"),
        MIDDLE("Mid");

        private final String str;
        Location(String text){
            str = text;
        }
    }

    public Location location;

    // ROIs
    static final Rect Left_ROI = new Rect(new Point(35, 125), new Point(85, 175));
    static final Rect Middle_ROI = new Rect(new Point(120, 100), new Point(205, 175));
    static final Rect Right_ROI = new Rect(new Point(235, 125), new Point(285, 175));

    public OpenCVDetector(Telemetry t) {
        telemetry = t;
    }

    double thresh = 0.5;

    @Override
    public Mat processFrame(Mat input) {

        Mat hsv = new Mat();
        Imgproc.cvtColor(input,hsv,Imgproc.COLOR_RGB2HSV);

        Scalar colMin = new Scalar(0,100,100);
        Scalar colMax = new Scalar(255,255,255);

        Core.inRange(hsv,colMin,colMax,hsv);

        Mat mid = hsv.submat(Left_ROI);
        Mat right = hsv.submat(Right_ROI);

        double midPer = Core.sumElems(mid).val[0] / Left_ROI.area() / 255;
        double rightPer = Core.sumElems(right).val[0] / Right_ROI.area() / 255;

        if(midPer > thresh){
            location = Location.MIDDLE;
        }
        else if(rightPer > thresh){
            location = Location.RIGHT;
        }
        else{
            location = Location.LEFT;
        }

        telemetry.addData("Mid", midPer);
        telemetry.addData("Right", rightPer);
        telemetry.addLine();
        telemetry.addData("Location", location.str);
        telemetry.update();

        Imgproc.rectangle(input, Left_ROI, new Scalar(0,255,0));
        Imgproc.rectangle(input, Right_ROI, new Scalar(0,255,0));

        mid.release();
        right.release();
        hsv.release();

        return input;
    }

}
