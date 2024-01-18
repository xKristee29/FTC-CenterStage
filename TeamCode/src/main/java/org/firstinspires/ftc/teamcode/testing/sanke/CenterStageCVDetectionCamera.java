package org.firstinspires.ftc.teamcode.testing.sanke;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.videoio.VideoCapture;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CenterStageCVDetectionCamera extends OpenCvPipeline {
    // Other constants and variables...
    Telemetry telemetry;
    VideoCapture camera;
    Mat mat = new Mat();

    public CenterStageCVDetectionCamera(Telemetry t) {
        telemetry = t;
        camera = new VideoCapture(0); // 0 for default camera
        if (!camera.isOpened()) {
            telemetry.addData("Error", "Could not open camera.");
            telemetry.update();
            return;
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        camera.read(mat);

        // Your existing image processing code here...

        return mat;
    }

    public void stopCamera() {
        if (camera != null) {
            camera.release();
        }
    }
}
