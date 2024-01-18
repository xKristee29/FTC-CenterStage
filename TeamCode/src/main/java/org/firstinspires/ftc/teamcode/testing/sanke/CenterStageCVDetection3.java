package org.firstinspires.ftc.teamcode.testing.sanke;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class CenterStageCVDetection3 extends OpenCvPipeline {
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
        LEFT,
        RIGHT,
        MIDDLE
    }

    public Location location;

    // ROIs
    //static final Rect Left_ROI = new Rect(10, 100, 105, 200);
    static final Rect Right_ROI = new Rect(120, 100, 205, 200);
    static final Rect Middle_ROI = new Rect(220, 100, 310, 200);

    public CenterStageCVDetection3(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        mat.release(); // Release the previous Mat to avoid memory leaks
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
        Mat mat1 = mat.clone();
        Mat mat2 = mat.clone();
        Mat mat3 = mat.clone();
        Mat mat4 = mat.clone();
        Core.inRange(mat1, new Scalar(0, 0, MINIMUM_VALUES), new Scalar(50, 50, MAXIMUM_VALUES), mat1); // Blue range
        Core.inRange(mat2, new Scalar(MINIMUM_VALUES, 0, 0), new Scalar(MAXIMUM_VALUES, 50, 50), mat2); // Red range
        Core.bitwise_or(mat1,mat2, mat);

        Core.inRange(mat3, new Scalar(0,0,100), new Scalar(50,50,255),mat3); // blue line
        Core.inRange(mat4, new Scalar(100,0,0), new Scalar(255,50,50),mat4); // red line

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
        if (rightValue==0 && middleValue==0) {
            location = Location.LEFT;
        }
        if (rightValue >= middleValue) {
            location = Location.RIGHT;
        } else {
            location = Location.MIDDLE;
        }

        switch (location) {
            case LEFT:
                telemetry.addData("Prop location", "Left");
                break;
            case MIDDLE:
                telemetry.addData("Prop Location", "Middle");
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
        Imgproc.rectangle(mat, Middle_ROI, location == Location.MIDDLE ? pixelColor : propColor);
        Imgproc.rectangle(mat, Right_ROI, location == Location.RIGHT ? pixelColor : propColor);

        return mat;
    }
}
