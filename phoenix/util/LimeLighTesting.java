package org.firstinspires.ftc.teamcode.phoenix.util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
@TeleOp(name = "Limelight OnBot TX", group = "Test")
public class LimeLighTesting extends OpMode {
    private Limelight3A limelight;
    
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
    }
    
    @Override
    public void start() {
        limelight.start();
    }
    
    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
           telemetry.addData("Tx", result.getTx());
           telemetry.addData("Ty", result.getTy());
        }
                      

            telemetry.update();
    }
}    