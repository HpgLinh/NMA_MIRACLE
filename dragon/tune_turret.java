package org.firstinspires.ftc.teamcode.dragon;
import org.firstinspires.ftc.teamcode.dragon.Turret;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "tune_turret")
public class tune_turret extends LinearOpMode{
    public Turret turret;
    @Override
    public void runOpMode() throws InterruptedException {
        turret = new Turret(hardwareMap);
        waitForStart();
        double targetdegree = 0;
        while(opModeIsActive()) {
            turret.setPosition(targetdegree);
            if(gamepad1.dpadRightWasPressed()) {
                targetdegree+=10;
            }
            if(gamepad1.dpadLeftWasPressed()) {
                targetdegree-=10;
            }
            telemetry.addLine("== KIỂM SOÁT TURRET THEO ĐỘ ==");
            telemetry.addData("Mục tiêu (Degree)", "%.1f°", targetdegree);
            telemetry.addData("Hiện tại (Degree)", "%.1f°", turret.currentDegree);
            telemetry.addData("Sai số (Degree)", "%.2f°", targetdegree - turret.currentDegree);

            telemetry.addLine("\n== THÔNG SỐ TUNING (PIDF) ==");
            telemetry.addData("Kp (Nhạy)", "%.4f (Nút Y/A để chỉnh)", turret.Kp);
            telemetry.addData("Kf (Ma sát)", "%.4f (Nút B/X để chỉnh)", turret.Kf);

            telemetry.update();
        }

    }

}