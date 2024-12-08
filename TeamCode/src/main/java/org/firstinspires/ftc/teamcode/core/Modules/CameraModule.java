package org.firstinspires.ftc.teamcode.core.Modules;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class CameraModule implements Module {
    VisionPortal camera;
    AprilTagProcessor aprilTagProcessor;
    boolean enabled = false;
    Pose distanceToTag = new Pose();

    public CameraModule(HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setLensIntrinsics(1385.92, 1385.92, 951.982, 534.084)
                .build();
        aprilTagProcessor.setDecimation(3);

        VisionPortal.Builder myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        myVisionPortalBuilder.setCameraResolution(new Size(1920, 1080));
        myVisionPortalBuilder.addProcessor(aprilTagProcessor);
        camera = myVisionPortalBuilder.build();

        camera.setProcessorEnabled(aprilTagProcessor, false);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        camera.setProcessorEnabled(aprilTagProcessor, enabled);
    }

    public Pose getDistanceToTag() {
        return distanceToTag;
    }

    @Override
    public void update() {
        if (enabled) {
            List<AprilTagDetection> rotatedDetection = aprilTagProcessor.getDetections();
            if (!rotatedDetection.isEmpty()) {
                for (AprilTagDetection detection : rotatedDetection)
                    if (detection.id == 10 || detection.id == 16)//TODO to change
                        distanceToTag = new Pose(detection.ftcPose);

            }
        }
    }
}
