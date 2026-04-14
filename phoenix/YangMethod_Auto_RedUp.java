package org.firstinspires.ftc.teamcode.phoenix;

import android.content.SharedPreferences;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.phoenix.geometry.Pose;
import org.firstinspires.ftc.teamcode.phoenix.localization.constants.TwoWheelConstants;
import org.firstinspires.ftc.teamcode.phoenix.localization.localizers.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.phoenix.util.Turret;

@Autonomous(name = "Dương's Method Auto Red Up", group = "Drive")
public class YangMethod_Auto_RedUp extends LinearOpMode {

    // --- Hardware Declarations ---
    private Turret turret = null;
    private DcMotorEx intakeDrive;
    private DcMotor backLeftMotor, frontLeftMotor, frontRightMotor, backRightMotor;
    private DcMotorEx shoot = null, shootstraff = null;
    private Servo srvLoad, srvIntake, srvOval;
    private IMU imu;

    // --- Localization ---
    private TwoWheelConstants twoWheelConstants;
    private TwoWheelLocalizer twoWheelLocalizer;
    private int rawX, rawY;

    // --- Variables & Timers ---
    private ElapsedTime timer = new ElapsedTime();
    private boolean wasTranslating = false;

    // --- PID Constants (Shooter) ---
    // double P = 50;
    // double I = 0.6;
    // double D = 0.001;
    // double F = 17;
    double P = 30;
    double I = 0.01;
    double D = 0;
    double F = 15;

    // --- Heading Hold PID ---
    // Error dùng RAD -> kp thường lớn hơn bản DEG
    double kp = 1;
    double ki = 0.0;
    double kd = 0.08;

    double targetHeading = 0.0; // rad
    double lastError = 0.0;
    double integral = 0.0;
    double botHeading;

    // --- Deadbands ---
    final double moveDeadband = 0.05;
    final double turnDeadband = 0.10;

    // =========================================================================
    // HELPER METHODS
    // =========================================================================

    private double angleWrap(double rad) {
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        while (rad < -Math.PI) rad += 2.0 * Math.PI;
        return rad;
    }

    private double clip01(double v) {
        return Range.clip(v, 0.0, 1.0);
    }

    void active_shoot(double ovalPos, int shootSpeed) {
        ElapsedTime waitTimer = new ElapsedTime();
        srvOval.setPosition(ovalPos);
        intakeDrive.setVelocity(3000);
        srvLoad.setPosition(0.36);
        shoot.setVelocity(shootSpeed);
        shootstraff.setVelocity(shootSpeed);
        while (waitTimer.seconds() <= 1.3 && opModeIsActive()) {
            moveDrvTrain(0, 0, 0);
        }
        srvLoad.setPosition(0.12);
        intakeDrive.setVelocity(4500);
    }
    
    void active_shoot1(double ovalPos, int shootSpeed) {
        ElapsedTime waitTimer = new ElapsedTime();
        srvOval.setPosition(ovalPos);
        intakeDrive.setVelocity(2500);
        srvLoad.setPosition(0.36);
        shoot.setVelocity(shootSpeed);
        shootstraff.setVelocity(shootSpeed);
        while (waitTimer.seconds() <= 1.7 && opModeIsActive()) {
            moveDrvTrain(0, 0, 0);
        }
        srvLoad.setPosition(0.08);
        intakeDrive.setVelocity(5000);
    }

    /**
     * Hàm điều khiển Chassis Mecanum với Heading Hold (Giữ hướng)
     */
    public void moveDrvTrain(double x, double y, double z) {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 0.02;

        // Deadband để tránh noise làm hỏng heading hold
        if (Math.abs(x) < moveDeadband) x = 0;
        if (Math.abs(y) < moveDeadband) y = 0;
        if (Math.abs(z) < turnDeadband) z = 0;

        // Reset heading khi bấm options
        if (gamepad1.options) {
            imu.resetYaw();
            targetHeading = 0.0;
            lastError = 0;
            integral = 0;
            while (gamepad1.options && opModeIsActive()) {
                // do nothing (debouncing)
            }
        }

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        boolean isTurning = (z != 0);
        boolean isTranslating = (Math.hypot(x, y) > 0);

        // --- ANTI-DRIFT CAPTURE ---
        // Khi vừa bắt đầu tịnh tiến mà không xoay
        if (isTranslating && !wasTranslating && !isTurning) {
            targetHeading = botHeading;
            lastError = 0;
            integral = 0;
        }
        wasTranslating = isTranslating;

        // --- HEADING HOLD OUTPUT ---
        double kpMove = kp;
        double kiMove = ki;
        double kdMove = kd;

        double kpStill = 1;      // bạn có thể thử 0.25 -> 0.5
        double kdStill = 0.03;

        double turnCmd;
        final double TURN_SIGN = -1;
        double stillDeadDeg = 0.3;

        if (isTurning) {
            // Manual turn
            turnCmd = -z;
            targetHeading = botHeading;
            lastError = 0;
            integral = 0;
        } else {
            double error = angleWrap(targetHeading - botHeading);
            double derivative = (error - lastError) / dt;
            double minTurn = isTranslating ? 0.03 : 0.05;

            if (isTranslating) {
                // Full PID while moving
                integral += error * dt;
                integral = Range.clip(integral, -0.3, 0.3);

                turnCmd = TURN_SIGN * (kpMove * error + kiMove * integral + kdMove * derivative);

                if (turnCmd != 0 && Math.abs(turnCmd) < minTurn) {
                    turnCmd = Math.signum(turnCmd) * minTurn;
                }
            } else {
                // Standing still
                integral = 0;
                if (Math.abs(error) > Math.toRadians(stillDeadDeg)) {
                    turnCmd = TURN_SIGN * (kpStill * error + kdStill * derivative);
                } else {
                    turnCmd = 0;
                }
                
                if (turnCmd != 0 && Math.abs(turnCmd) < minTurn) {
                    turnCmd = Math.signum(turnCmd) * minTurn;
                }
            }

            turnCmd = Range.clip(turnCmd, -0.6, 0.6);
            if (turnCmd != 0 && Math.abs(turnCmd) < minTurn) {
                turnCmd = Math.signum(turnCmd) * minTurn;
            }

            lastError = error;
        }

        // --- FIELD CENTRIC ---
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX *= 1.1; // Counteract imperfect strafing

        // --- MECANUM POWER CALCULATION ---
        double turnWeight = (isTranslating && !isTurning) ? 0.35 : 1.0;
        double denominator = Math.max(
                Math.abs(rotY) + Math.abs(rotX) + Math.abs(turnCmd) * turnWeight, 1
        );

        boolean check = (gamepad1.right_trigger > 0.1);
        double speedScale = check ? 0.5 : 1.0;

        double frontLeftPower  = (rotY + rotX + turnCmd) / denominator * speedScale;
        double backLeftPower   = (rotY - rotX + turnCmd) / denominator * speedScale;
        double frontRightPower = (rotY - rotX - turnCmd) / denominator * speedScale;
        double backRightPower  = (rotY + rotX - turnCmd) / denominator * speedScale;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        // --- ODOMETRY & TELEMETRY ---
        rawY = frontLeftMotor.getCurrentPosition();
        rawX = backRightMotor.getCurrentPosition();
        
        // Sân xanh logic
        twoWheelLocalizer.update();
        Pose pose = twoWheelLocalizer.getPose();
        double angle_unit = Math.atan2(135 - pose.getY(), 135 - pose.getX()) - pose.getHeading();
        
        while (angle_unit > Math.PI) angle_unit -= 2 * Math.PI;
        while (angle_unit <= -Math.PI) angle_unit += 2 * Math.PI;
        
        turret.setPosition((Math.toDegrees(angle_unit)));

        telemetry.addData("foward encoder", frontLeftMotor.getCurrentPosition());
        telemetry.addData("strafe encoder", backRightMotor.getCurrentPosition());
        telemetry.addData("angle unit", Math.toDegrees(angle_unit));
        telemetry.addData("atan2", Math.atan2(135 - pose.getY(), 10 - pose.getX()));
        telemetry.addData("y", pose.getY());
        telemetry.addData("x", pose.getX());
        telemetry.addData("Heading rad", botHeading);
        telemetry.addData("Heading deg", (Math.toDegrees(botHeading) + 360) % 360);
        telemetry.addData("raw_y", rawY);
        telemetry.addData("raw_x", rawX);
        telemetry.addData("shoot motor", shoot.getVelocity());
        telemetry.addData("Target deg", Math.toDegrees(targetHeading));
        telemetry.addData("Turning", isTurning);
        telemetry.addData("turnCmd", turnCmd);
        telemetry.addData("intakeEncoder", intakeDrive.getCurrentPosition());
        telemetry.update();
    }

    public void step0() {
        // --- Giai đoạn 0: Chéo về vị trí bắn ---
        // Khởi động Shooter
        // shoot.setVelocity(1600);
        // shootstraff.setVelocity(1600);
        shoot.setVelocity(1800);
        shootstraff.setVelocity(1800);

        // --- Di chuyển tới tuyến bắn ---
        while (rawX > -10000 && opModeIsActive()) {
            moveDrvTrain(1, -0.6, 0);
        }

        active_shoot(0.06, 1750);
    }

    public void step1() {
        // ---Giai đoạn 1: Di chuyển tới hàng 2 và intake ---
        // --- Đi ngang về hàng 2 ---
        while (rawX > -32500 && opModeIsActive()) {
            moveDrvTrain(1, 0, 0);
        }

        // --- Vào Intake hàng 2 ---
        while (rawY < 9000 && opModeIsActive()) {
            moveDrvTrain(0, 0.7, 0);
        }

        // --- Lùi ra khỏi hàng 2 ---
        while (rawY > 7000 && opModeIsActive()) {
            moveDrvTrain(0, -1, 0);
        }
        while (rawY > -2000 && opModeIsActive()) {
            moveDrvTrain(-0.8, -1, 0);
        }

        // --- Di chuyển tới vị trí bắn ---
        while (rawX < -24000 && opModeIsActive()) {
            moveDrvTrain(-1, 0, 0);
        }

        active_shoot(0.06, 1750);
    }

    public void step2() {
        // --- Giai đoạn 2: Di chuyển tới hàng 2 lần nữa và intake ---
        // --- Đi ngang về hàng 2 ---
        while (rawX > -32000 && opModeIsActive()) {
            moveDrvTrain(1, 0, 0);
        }
        
        ElapsedTime time0 = new ElapsedTime();

        // --- Tiến vào gạt cần ---
        while (Math.abs(Math.toDegrees(botHeading)) < 27 && opModeIsActive()) {
            moveDrvTrain(0, 1, 0.4);
        }

        while (rawY < 6100 && opModeIsActive()) {
            moveDrvTrain(0, 0.6, 0);
        }
    
        time0.reset();
        while (time0.seconds() <= 0.5 && opModeIsActive()) {
            moveDrvTrain(0, 0, 0);
        }
        
        // --- trượt ngang để đón bóng ---
        time0.reset();
        while (time0.seconds() <= 0.4 && opModeIsActive())
        {
            moveDrvTrain(0.4, 0.3, 0);
        }
        
        time0.reset();
        while (time0.seconds() <= 1 && opModeIsActive()) {
            moveDrvTrain(0, 0, 0);
        }
        
        // --- lui ra ---
        time0.reset();
        while (time0.seconds() <= 0.4 && opModeIsActive()) {
            moveDrvTrain(-0.4, -1, 0);
            // moveDrvTrain(0, 0, 0.3)
        }

        // -- vào lại --
        time0.reset();
        while (time0.seconds() <= 0.5 && opModeIsActive()) {
            moveDrvTrain(-0.05, 1, 0);
        }
        
        time0.reset();
        while (time0.seconds() <= 0.2 && opModeIsActive())
        {
            moveDrvTrain(0.3, 0.3, 0);
        }
        
        time0.reset();
        while (time0.seconds() <= 0.2 && opModeIsActive()) {
            moveDrvTrain(0, 0, 0);
        }
        
        // --- lui ra lần 2 ---
        time0.reset();
        while (time0.seconds() <= 0.2 && opModeIsActive()) {
            moveDrvTrain(-0, -1, 0);
            // moveDrvTrain(0, 0, 0.3)
        }
        
        // -- vào lại lần 2--
        time0.reset();
        while (time0.seconds() <= 0.5 && opModeIsActive()) {
            moveDrvTrain(-0.2, 1, 0);
        }
        
        time0.reset();
        while (time0.seconds() <= 0.2 && opModeIsActive())
        {
            moveDrvTrain(0.3, 0.3, 0);
        }
        
        time0.reset();
        while (time0.seconds() <= 0.5 && opModeIsActive()) {
            moveDrvTrain(0, 0, 0);
        }

        // -- thoát khỏi hàng --
        while (rawY > 0 && opModeIsActive()) {
            moveDrvTrain(0, -1, 0);
        }
 
        while (Math.toDegrees(botHeading) > 0 && opModeIsActive()) {
            moveDrvTrain(0, -1, -0.7);
        }
        
        // --- Di chuyển để bắn tiếp ---
        while (rawX < -25000 && opModeIsActive()) {
            moveDrvTrain(-1, 0, 0);
        }

        active_shoot(0.07, 1750);
    }

    public void step3() {
        // --- Giai đoạn 3: Di chuyển tới hàng 1 và intake ---
        while (rawX > -23000 && opModeIsActive()) {
            moveDrvTrain(1, 0, 0);
        }
        while (rawY < 5800 && opModeIsActive()) {
            moveDrvTrain(0, 0.7, 0);
        }

        while (rawY > -2000 && opModeIsActive()) {
            moveDrvTrain(0, -1, 0);
        }

        active_shoot(0.075, 1800);
    }

    public void step4() {
        // --- Giai đoạn 4: Di chuyển tới hàng 3 và intake ---
        while (rawX > -46000 && opModeIsActive()) {
            moveDrvTrain(1, 0, 0);
        }

        while (rawY < 8300 && opModeIsActive()) {
            moveDrvTrain(0, 0.8, 0);
        }

        while (rawY > -0 && opModeIsActive()) {
            moveDrvTrain(0, -1, 0);
        }

        while (rawX < -30000 && opModeIsActive()) {
            moveDrvTrain(-1, 0, 0);
        }

        active_shoot1(0.05, 1800);
    }

    public void step5() {
        ElapsedTime time0 = new ElapsedTime();
        time0.reset();
        while(time0.seconds() <= 0.5 && opModeIsActive()) moveDrvTrain(0.4, 1, 0);
    }
    // =========================================================================
    // MAIN OP MODE
    // =========================================================================
    @Override
    public void runOpMode() throws InterruptedException {
        SharedPreferences prefs =
                hardwareMap.appContext.getSharedPreferences("FTC_DATA0", 0);
        SharedPreferences.Editor editor = prefs.edit();

        // -----------------------------
        // 0) HARDWARE MAP
        // -----------------------------
        twoWheelConstants = new TwoWheelConstants();
        twoWheelLocalizer = new TwoWheelLocalizer(hardwareMap, twoWheelConstants, new Pose(112, 135, Math.toRadians(0)));

        backLeftMotor   = hardwareMap.dcMotor.get("drv3");
        frontLeftMotor  = hardwareMap.dcMotor.get("drv2");
        frontRightMotor = hardwareMap.dcMotor.get("drv1");
        backRightMotor  = hardwareMap.dcMotor.get("drv4");

        // Motor Setup
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servo & Other Motors
        Servo srvTurret1 = hardwareMap.get(Servo.class, "srvTurret0");
        Servo srvTurret2 = hardwareMap.get(Servo.class, "srvTurret1");
        intakeDrive = hardwareMap.get(DcMotorEx.class, "drvintake");
        srvOval     = hardwareMap.get(Servo.class, "srvOval0");
        srvIntake   = hardwareMap.get(Servo.class, "srvIntake");
        srvLoad     = hardwareMap.get(Servo.class, "srvLoad");
        shoot       = hardwareMap.get(DcMotorEx.class, "drvShoot");
        shootstraff = hardwareMap.get(DcMotorEx.class, "drvShootstraff");
        
        CRServo srvblock0 = hardwareMap.get(CRServo.class, "srvblock0");
        CRServo srvblock1 = hardwareMap.get(CRServo.class, "srvblock1");

        // Shooter Setup
        shoot.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shoot.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        
        intakeDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        
        shootstraff.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shootstraff.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shoot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shootstraff.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shoot.setVelocityPIDFCoefficients(P, I, D, F);
        shootstraff.setVelocityPIDFCoefficients(P, I, D, F);
        
        turret = new Turret(hardwareMap);

        // Giữ đúng hướng như code gốc của bạn
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // -----------------------------
        // 1) IMU INIT
        // -----------------------------
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();
        sleep(300);

        waitForStart();

        // -----------------------------
        // START OF AUTONOMOUS
        // -----------------------------
        
        // Servo init positions
        srvIntake.setPosition(0.25);
        srvLoad.setPosition(0.1);
        srvOval.setPosition(0.08);
        srvblock0.setPower(-1);
        srvblock1.setPower(-1);
        intakeDrive.setVelocity(5000);

        // Đồng bộ targetHeading ngay sau resetYaw
        targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        lastError = 0;
        integral = 0;
        wasTranslating = false;

        if (isStopRequested()) return;

        timer.reset();

        step0();
        step1();
        step2();
        step3();
        step4();
        step5();

        twoWheelLocalizer.update();
        Pose pose = twoWheelLocalizer.getPose();

        float saveX = (float)pose.getX();
        float saveY = (float)pose.getY();
        float saveZ = (float)pose.getHeading();
        float turret_degree = (float)turret.currentTicks;
        editor.putFloat("x", saveX);
        editor.putFloat("y", saveY);
        editor.putFloat("heading", saveZ);
        editor.putFloat("turret degree", turret_degree);
        editor.apply();
        
        while (opModeIsActive())
        {
            moveDrvTrain(0, 0, 0);
            telemetry.addData("x", saveX);
            telemetry.addData("y", saveY);
            telemetry.addData("heading", saveZ);
            telemetry.update();
        }
    }
}