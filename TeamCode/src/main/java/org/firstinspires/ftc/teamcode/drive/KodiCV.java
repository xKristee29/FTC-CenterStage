package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class KodiCV {

    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    private OpenCvCamera webcam = null;
    public OpenCVDetector detector = null;
    private int cameraMonitorViewId;
    private boolean isRunning = true;

    public KodiCV(Telemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

    }

    public void start(){
        detector = new OpenCVDetector(telemetry);
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Error opening camera",errorCode);
                isRunning = false;
            }
        });
        FtcDashboard.getInstance().startCameraStream(webcam, 30);
    }

    public boolean isRunning(){ return isRunning;}

    public void kill(){
        if(webcam != null) webcam.stopStreaming();
    }

    public int getFrameCount(){ return webcam.getFrameCount();}

    public float getFps(){ return webcam.getFps();}

    public int getTotalFrameTimeMs(){ return webcam.getTotalFrameTimeMs();}

    public int getPipelineTimeMs(){ return webcam.getPipelineTimeMs();}

    public int getOverheadTimeMs(){ return webcam.getOverheadTimeMs();}

    public int getCurrentPipelineMaxFps(){ return webcam.getCurrentPipelineMaxFps();}
}
