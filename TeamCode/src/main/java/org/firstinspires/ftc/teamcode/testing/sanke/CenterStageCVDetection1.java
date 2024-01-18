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

@Config
public class CenterStageCVDetection1 extends OpenCvPipeline{
    //Values
    public static boolean DETECT_RED = false;
    public static double MINIMUM_VALUES = 100;
    public static double MAXIMUM_VALUES = 255;
    public static double MINIMUM_BLUE_HUE = 100;
    public static double MAXIMUM_BLUE_HUE = 115;
    public static double MINIMUM_RED_LOW_HUE = 0;
    public static double MAXIMUM_RED_LOW_HUE = 25;
    public static double MINIMUM_RED_HIGH_HUE = 160;
    public static double MAXIMUM_RED_HIGH_HUE = 255;
    Telemetry telemetry;
    Mat mat = new Mat();


    public enum Location {
        Left,
        Right,
        Middle
    }
    public Location location;

    //ROIs
    static final Rect Left_ROI = new Rect(new Point(10,100), new Point(105,200));
    static final Rect Right_ROI = new Rect(new Point(120,100), new Point(205,200));
    static final Rect Middle_ROI = new Rect(new Point(220,100), new Point(310,200));

    public CenterStageCVDetection1(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar MINIMUM_BLUE = new Scalar(MINIMUM_BLUE_HUE,MINIMUM_VALUES,MINIMUM_VALUES);
        Scalar MAXIMUM_BLUE = new Scalar(MAXIMUM_BLUE_HUE, MAXIMUM_VALUES, MAXIMUM_VALUES);
        Scalar MINIMUM_RED_LOW = new Scalar(MINIMUM_RED_LOW_HUE, MINIMUM_VALUES,MINIMUM_VALUES);
        Scalar MAXIMUM_RED_LOW = new Scalar(MAXIMUM_RED_LOW_HUE,MAXIMUM_VALUES,MAXIMUM_VALUES);
        Scalar MINIMUM_RED_HIGH = new Scalar(MINIMUM_RED_HIGH_HUE,MINIMUM_VALUES, MINIMUM_VALUES);
        Scalar MAXIMUM_RED_HIGH = new Scalar(MAXIMUM_RED_HIGH_HUE,MAXIMUM_VALUES,MAXIMUM_VALUES);

        if (!DETECT_RED)
            Core.inRange(mat, MINIMUM_BLUE, MAXIMUM_BLUE, mat); //Blue value
        else {
            //Red value
            Mat mat1 = mat.clone();
            Mat mat2 = mat.clone();
            Core.inRange(mat1, MINIMUM_RED_LOW, MAXIMUM_RED_LOW, mat1);
            Core.inRange(mat2, MINIMUM_RED_HIGH, MAXIMUM_RED_HIGH, mat2);
            Core.bitwise_or(mat1, mat2, mat);
        }
        //submat = submatrix - proportion of original matrix
        Mat left = mat.submat(Left_ROI);
        Mat right = mat.submat(Right_ROI);
        Mat middle = mat.submat(Middle_ROI);


        double leftValue = Core.sumElems(left).val[0];
        double rightValue = Core.sumElems(right).val[0];
        double middleValue = Core.sumElems(middle).val[0];

        //telemetrie
        telemetry.addData("Left Raw Value", leftValue);
        telemetry.addData("Right Raw Data", rightValue);
        telemetry.addData("Middle Raw Data", middleValue);

        left.release();
        right.release();
        middle.release();

        //get location
        if (leftValue >= rightValue && leftValue >= middleValue) {
            location = Location.Left;
            telemetry.addData("Prop location", "Right");
        } else if (rightValue >= middleValue) {
            location = Location.Right;
            telemetry.addData("Prop location", "Left");
        } else {
            location = Location.Middle;
            telemetry.addData("Prop Location", "Middle");
        }

        //update la telemetrie
        telemetry.update();

        //drawing rectangles ca sa vedem obiectele
        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);
        Scalar pixelColor = new Scalar(255,255,255);
        Scalar propColor = new Scalar(0,0,255);

        Imgproc.rectangle(mat, Left_ROI, location == Location.Left? pixelColor:propColor);
        Imgproc.rectangle(mat, Middle_ROI, location == Location.Middle? pixelColor:propColor);
        Imgproc.rectangle(mat, Right_ROI, location == Location.Right? pixelColor:propColor);

        return mat;
    }

}
