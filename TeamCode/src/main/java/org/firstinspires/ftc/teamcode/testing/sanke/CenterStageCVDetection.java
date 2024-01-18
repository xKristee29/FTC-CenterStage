package org.firstinspires.ftc.teamcode.testing.sanke;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class CenterStageCVDetection extends OpenCvPipeline {
    // Values
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

    public CenterStageCVDetection(Telemetry t) {
        telemetry = t;
    }

    double threshold = 0.5;

    @Override
    public Mat processFrame(Mat input){

        Mat mid = input.submat(Left_ROI);
        Mat right = input.submat(Right_ROI);

        telemetry.addData("H",input.rows());
        telemetry.addData("W",input.cols());
        telemetry.addLine();

        List<Mat> chanMid = new ArrayList<Mat>(), chanRight = new ArrayList<Mat>();
        Core.split(mid, chanMid);
        Core.split(right, chanRight);

        double redMid = Core.sumElems(chanMid.get(0)).val[0] / 255 / Left_ROI.area();
        double redRight = Core.sumElems(chanRight.get(0)).val[0] / 255 / Right_ROI.area();

        double blueMid = Core.sumElems(chanMid.get(2)).val[0] / 255 / Left_ROI.area();
        double blueRight = Core.sumElems(chanRight.get(2)).val[0] / 255 / Right_ROI.area();

        boolean midProp = redMid > threshold || blueMid > threshold;
        boolean rightProp = redRight > threshold || blueRight > threshold;

        if(midProp){
            location = Location.MIDDLE;
        }
        else if(rightProp){
            location = Location.RIGHT;
        }
        else{
            location = Location.LEFT;
        }

        telemetry.addData("Mid Red",redMid);
        telemetry.addData("Right Red",redRight);
        telemetry.addData("Mid Blue",blueMid);
        telemetry.addData("Right Blue",blueRight);
        telemetry.addLine();
        telemetry.addData("Location", location.str);
        telemetry.update();

        Imgproc.rectangle(input, Left_ROI, new Scalar(255,255,255));
        Imgproc.rectangle(input, Right_ROI, new Scalar(255,255,255));

        return input;
    }


    public Mat processFrame2(Mat input) {
        mat = input.clone(); // Clone the input Mat
        //Varianta fara Left_ROI


        /*
        if (!DETECT_RED) {
            //blue values
            Core.inRange(mat, new Scalar(MINIMUM_BLUE_RED, MINIMUM_VALUES, MINIMUM_VALUES),
                    new Scalar(MAXIMUM_BLUE_RED, MAXIMUM_VALUES, MAXIMUM_VALUES), mat);
        } else {
            // Red value
            Core.inRange(mat, new Scalar(0, 0, 0), new Scalar(25, MAXIMUM_VALUES, MAXIMUM_VALUES), mat);
        }
         */


        //ne uitam dupa ambele culori (needz to be tested)
        //Core.inRange(mat, new Scalar(MINIMUM_BLUE_RED, MINIMUM_VALUES, MINIMUM_VALUES), new Scalar(MAXIMUM_BLUE_RED, MAXIMUM_VALUES, MAXIMUM_VALUES), mat);
        Mat mat1 = null, mat2 = null;
        Core.inRange(mat, new Scalar(0, 0, 100), new Scalar(50, 50, 255), mat1); // Blue range
        Core.inRange(mat, new Scalar(100, 0, 0), new Scalar(255, 50, 50), mat2); // Red range
        Core.add(mat1,mat2,mat);

        // Submatrices
        //Mat left = mat.submat(Left_ROI);
        Mat right = mat.submat(Right_ROI);
        Mat middle = mat.submat(Middle_ROI);

        // Sum of pixels
        //double leftValue = Core.sumElems(left).val[0];
        double rightValue = Core.sumElems(right).val[0];
        double middleValue = Core.sumElems(middle).val[0];

        // Telemetrie
        //telemetry.addData("Left Raw Value", leftValue);
        telemetry.addData("Right Raw Data", rightValue);
        telemetry.addData("Middle Raw Data", middleValue);

        //left.release();
        right.release();
        middle.release();

        // location
        location = Location.LEFT;

        if (rightValue >= middleValue) {
            location = Location.RIGHT;
        } else {
            location = Location.MIDDLE;
        }

        switch (location){
            case LEFT:
                telemetry.addData("Prop location", "Left");
                break;
            case MIDDLE:
                telemetry.addData("Prop location", "Mid");
                break;
            case RIGHT:
                telemetry.addData("Prop location", "Right");
                break;
        }

        // Update la telemetrie
        telemetry.update();

        // Desenam rectangles pentru o vizualizare a prop-ului mai buna
        Scalar pixelColor = DETECT_RED ? RED : BLUE;
        Scalar propColor = new Scalar(0, 0, 255);

        // Imgproc.rectangle(mat, Left_ROI, location == Location.Left ? pixelColor : propColor);
        Imgproc.rectangle(input, Middle_ROI, location == Location.MIDDLE ? pixelColor : propColor);
        Imgproc.rectangle(input, Right_ROI, location == Location.RIGHT ? pixelColor : propColor);

        mat.release(); // Release the previous Mat to avoid memory leaks
        mat1.release(); // Release the previous Mat to avoid memory leaks
        mat2.release(); // Release the previous Mat to avoid memory leaks


        return input;
    }
}