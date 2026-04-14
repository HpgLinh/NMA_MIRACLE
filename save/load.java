import android.content.SharedPreferences;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="TeleLoadData")
public class load extends LinearOpMode {

    @Override
    public void runOpMode() {

        SharedPreferences prefs =
                hardwareMap.appContext.getSharedPreferences("FTC_DATA", 0);

        double x = prefs.getFloat("x", 0f);
        double y = prefs.getFloat("y", 0f);
        double heading = prefs.getFloat("heading", 0f);

        telemetry.addLine("Loaded from Auto:");
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("heading (deg)", Math.toDegrees(heading));
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Dùng dữ liệu này để tiếp tục điều khiển
        }
    }
}
