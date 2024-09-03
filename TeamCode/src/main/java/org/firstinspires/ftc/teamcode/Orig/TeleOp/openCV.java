package org.firstinspires.ftc.teamcode.Orig.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "TEST OPEN CV", group = "Test")
public class openCV extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        // Add telemetry to track the initialization process
        telemetry.addData("Status", "Initializing Camera");
        telemetry.update();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addData("Status", "Camera Opened Successfully");
                telemetry.update();

                // Start streaming at a standard resolution
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Error Code: " + errorCode);
                telemetry.update();
            }
        });

        // Initialize the detector pipeline
        OpenCVCameraTest detector = new OpenCVCameraTest(telemetry);
        camera.setPipeline(detector);

        waitForStart();

        while (opModeIsActive()) {
            // Add telemetry to check if frames are being processed
            telemetry.addData("Frame Count", camera.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", camera.getFps()));
            telemetry.addData("Total frame time ms", camera.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", camera.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", camera.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", camera.getCurrentPipelineMaxFps());

            // Check and display the detector results
            switch (detector.getLocation()) {
                case LEFT:
                    telemetry.addData("Detected", "Left");
                    break;
                case MIDDLE:
                    telemetry.addData("Detected", "Middle");
                    break;
                case RIGHT:
                    telemetry.addData("Detected", "Right");
                    break;
                default:
                    telemetry.addData("Detected", "Unknown");
                    break;
            }

            telemetry.update();
            sleep(100);  // Add a small delay to avoid overwhelming the telemetry
        }

        // Stop the camera streaming once the op mode stops
        camera.stopStreaming();
    }
}
