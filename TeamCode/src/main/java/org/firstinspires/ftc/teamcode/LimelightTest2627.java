package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "LimelightTest2627")
public class LimelightTest2627 extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // 1. Map the hardware using the exact name from your Driver Station config
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // 2. Set polling rate (Hz) and select your pipeline index (0-9)
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);

        // 3. Start polling for vision data
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            // 4. Fetch the latest vision frame
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx(); // Horizontal offset (degrees)
                double ty = result.getTy(); // Vertical offset (degrees)
                double ta = result.getTa(); // Target area (% of image)

                telemetry.addData("Current iter:", "establishing github directory");
                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No valid target");
            }
            telemetry.update();
        }

        // 5. Stop polling when the OpMode ends
        limelight.stop();
    }
}
