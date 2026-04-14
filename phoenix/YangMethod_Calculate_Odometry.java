package org.firstinspires.ftc.teamcode.phoenix;



import org.firstinspires.ftc.teamcode.dragon.Turret;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.phoenix.localization.localizers.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.phoenix.localization.constants.TwoWheelConstants;
import org.firstinspires.ftc.teamcode.phoenix.geometry.Pose;

@TeleOp(name = "Dương's Method Calculate Odometry", group = "Drive")
public class YangMethod_Calculate_Odometry extends LinearOpMode {
    int rawX;
    int rawY;
    ElapsedTime timer = new ElapsedTime();
    
    
    
     DcMotor intakeDrive;
    DcMotor backLeftMotor;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    Servo srvLoad, srvIntake;
    TwoWheelConstants twoWheelConstants; 
    TwoWheelLocalizer twoWheelLocalizer;
    double P = 50;
    double I = 0.6;
    double D = 0.001;
    double F = 17;
    // ---------- Heading Hold PID ----------
    // Bạn có thể tune lại 3 giá trị này    (TAO BẢO MÀY TUNE MÀY LẠI BẢO TAO TUNE? AI LÀ BỐ? TRẢ LỜI?)
    double kp = 1;      // vì error đang dùng RAD -> kp thường lớn hơn bản DEG (THẾ TAO VỚI MÀY AI LỚN HƠN?)
    double ki = 0.0;
    double kd = 0.08;

    double targetHeading = 0.0;  // rad
    double lastError = 0.0;
    double integral = 0.0;
    double botHeading;
    private DcMotorEx shoot = null, shootstraff = null;

    private Servo srvOval = null;

    boolean wasTranslating = false;
    IMU imu;
    
    
    
    
    double startTimeDelay;
    // Deadbands
    final double moveDeadband = 0.05;
    final double turnDeadband = 0.10;

    private double angleWrap(double rad) {
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        while (rad < -Math.PI) rad += 2.0 * Math.PI;
        return rad;
    }
    
    void active_shoot(){
        ElapsedTime waitTimer = new ElapsedTime();
        intakeDrive.setPower(1);
        srvLoad.setPosition(0.35);
        while(waitTimer.seconds() <= 1.5 && opModeIsActive()) moveDrvTrain(0, 0, 0);
        srvLoad.setPosition(0.08);   
        intakeDrive.setPower(0.65);
    }

    // -----------------------------
    private double clip01(double v) {
        return Range.clip(v, 0.0, 1.0);
    }
    
    public void moveDrvTrain(double x, double y, double z)
    {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 0.02;
        
        // Deadband để tránh noise làm hỏng heading hold (NHƯNG BỐ MÀY THÍCH HỎNG, Ý KIẾN?)
        if (Math.abs(x) < moveDeadband) x = 0;
        if (Math.abs(y) < moveDeadband) y = 0;
        if (Math.abs(z) < turnDeadband) z = 0;

        // Reset heading khi bấm options (TAO MUỐN RESET HEADING KHI ĐẬP MẸ CON BOT ĐẤY, MÀY CÓ LÀM ĐƯỢC KO?)
        if (gamepad1.options) {
            imu.resetYaw();
            targetHeading = 0.0;
            lastError = 0;
            integral = 0;

            while (gamepad1.options && opModeIsActive()) {
                ; // do nothing (CHÍNH XÁC NHƯ TÁC DỤNG CỦA MÀY, DO NOTHING)
            }
        }

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        boolean isTurning = (z != 0);
        boolean isTranslating = (Math.hypot(x, y) > 0);

        // -----------------------------
        // 3) ANTI-DRIFT CAPTURE
        // Khi vừa bắt đầu tịnh tiến mà không xoay (TAO MUỐN XOAY MUỐN GIÃY ĐÀNH ĐẠCH LÊN ĐẤY)
        // -----------------------------
        if (isTranslating && !wasTranslating && !isTurning) {
            targetHeading = botHeading;
            lastError = 0;
            integral = 0;
        }
        wasTranslating = isTranslating;

        // -----------------------------
        // 4) HEADING HOLD OUTPUT (ANH KHÔNG RA ANH VIỆT KHÔNG RA VIỆT, MÀY LÀ NGƯỜI CALI GỐC VIỆT À)
        // -----------------------------
        double kpMove = kp;
        double kiMove = ki;
        double kdMove = kd;

        double kpStill = 1;      // bạn có thể thử 0.25 -> 0.5 (BỐ MÀY ĐÉO THÍCH?)
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
            //if(Math.abs(error) < 1) break; (KHÔNG DÙNG THÌ COMMENT ĂN CỨT À, NHIỄU)
            double derivative = (error - lastError) / dt;

            double minTurn = isTranslating ? 0.03 : 0.05;

            if (isTranslating) {
                // Full PID while moving    (KHÔNG PID KHI MOVING CHẢ LẼ PID KHI ĐỨNG YÊN?)
                integral += error * dt;
                integral = Range.clip(integral, -0.3, 0.3);

                turnCmd = TURN_SIGN * (kpMove * error + kiMove * integral + kdMove * derivative);

                if (turnCmd != 0 && Math.abs(turnCmd) < minTurn) {
                    turnCmd = Math.signum(turnCmd) * minTurn;
                }

            } else {
                // Standing still:
                // - no integral
                // - large deadband so drift won't cause twitch
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

        // -----------------------------
        // 5) FIELD CENTRIC (TAO ĐÉO THÍCH FIELD CENTRIC, TAO THÍCH EARTH CENTRIC ĐẤY)
        // -----------------------------
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX *= 1.1; // Counteract imperfect strafing

        // -----------------------------
        // 6) MECANUM POWER (SỨC MẠNH CỦA MECANUM?)
        // -----------------------------
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
        


        // twoWheelLocalizer.update();
        // Pose pose = twoWheelLocalizer.getPose();

        
        
        rawY = frontLeftMotor.getCurrentPosition();
        rawX = -backRightMotor.getCurrentPosition();
        telemetry.addData("foward encoder", frontLeftMotor.getCurrentPosition());
        telemetry.addData("strafe encoder", backRightMotor.getCurrentPosition());
        
        
        // sân xanh
        twoWheelLocalizer.update();
        Pose pose = twoWheelLocalizer.getPose();
        double angle_unit = Math.atan2(135-pose.getY(),10 - pose.getX()) - pose.getHeading();
        while (angle_unit > Math.PI) angle_unit -= 2 * Math.PI;
        while (angle_unit <= -Math.PI) angle_unit += 2 * Math.PI;

        
        telemetry.addData("angle unit", Math.toDegrees(angle_unit));
        telemetry.addData("atan2", Math.atan2(135-pose.getY(),10 - pose.getX()));
        telemetry.addData("y", pose.getY());
        telemetry.addData("x", pose.getX());
        telemetry.addData("Heading rad", botHeading);
        telemetry.addData("Heading deg", (Math.toDegrees(botHeading) + 360) % 360);
        telemetry.addData("raw_y",rawY);
        telemetry.addData("raw_x",rawX);
        telemetry.addData("shoot motor", shoot.getVelocity());
        // telemetry.addData("State", stateIntake);
        telemetry.addData("Target deg", Math.toDegrees(targetHeading));
        telemetry.addData("Turning", isTurning);
        telemetry.addData("turnCmd", turnCmd);
        telemetry.update();
    }
    
    @Override
    public void runOpMode() throws InterruptedException {

        // -----------------------------
        // 0) HARDWARE MAP (MAP CÁI ĐỊT CON MẸ, CODE GPT ĂN ĐẦU BUỒI, ĂN CỨT)
        // -----------------------------
        twoWheelConstants = new TwoWheelConstants();
        twoWheelLocalizer = new TwoWheelLocalizer(hardwareMap, twoWheelConstants, new Pose(32,135,Math.toRadians(180)));

        backLeftMotor  = hardwareMap.dcMotor.get("drv3");
        frontLeftMotor = hardwareMap.dcMotor.get("drv2");
        frontRightMotor = hardwareMap.dcMotor.get("drv1");
        backRightMotor = hardwareMap.dcMotor.get("drv4");

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.COAST);
        // frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Servo srvTurret1 = hardwareMap.get(Servo.class, "srvTurret0");
        Servo srvTurret2 = hardwareMap.get(Servo.class, "srvTurret1");
        intakeDrive = hardwareMap.get(DcMotor.class, "drvintake");
        srvOval = hardwareMap.get(Servo.class, "srvOval0");
        srvIntake = hardwareMap.get(Servo.class, "srvIntake");
        srvLoad = hardwareMap.get(Servo.class, "srvLoad");
        shoot = hardwareMap.get(DcMotorEx.class, "drvShoot");
        shootstraff = hardwareMap.get(DcMotorEx.class, "drvShootstraff");
        CRServo srvblock0 = hardwareMap.get(CRServo.class, "srvblock0");
        CRServo srvblock1 = hardwareMap.get(CRServo.class, "srvblock1");
        
        shoot.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shoot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shootstraff.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shootstraff.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shoot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shootstraff.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); //

        shoot.setVelocityPIDFCoefficients(P, I, D, F);
        shootstraff.setVelocityPIDFCoefficients(P, I, D, F);

        

        int stateIntake = 0;
        int power = 1200;


        // Giữ đúng hướng như code gốc của bạn(BẠN BÈ ĐÉO GÌ MÀY)
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // -----------------------------
        // 1) IMU INIT (TAO BIẾT ĐỌC, OK?)
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
        // while(opModeInInit()) {
        //     double y = -gamepad1.left_stick_y;
        //     double x = gamepad1.left_stick_x;
        //     double rx = gamepad1.right_stick_x;
        //     moveDrvTrain(x, y, rx);
        // };
        waitForStart();

        // Reset yaw theo thói quen code gốc (THÓI QUEN LOZ NÀO, TÍNH BẮT BÀI TAO À, TUỔI LOZ)
        

        // Servo init

        // Đồng bộ targetHeading ngay sau resetYaw (AI HỎI MÀ BỘ TRƯỞNG TRẢ LỜI?)
        targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        lastError = 0;
        integral = 0;
        boolean prevX=false, prevB=false, prevLB=false, prevRB=false;
        boolean prevOpt=false, prevShare = false;
        boolean prevDpadUp=false, prevDpadDown=false, prevDpadLeft=false, prevDpadRight=false;
        boolean prevRT=false, prevLT=false;
        boolean prevA=false, prevY=false;
        boolean intakeDeployed = false; // false -> stow (CÁI LOZ GÌ ĐÂY? AM I A FUCKING JOKE 2 U?)
        boolean loadOpen = false;
        wasTranslating = false;

        if (isStopRequested()) return;

        
        timer.reset();
        



        
        // -----------------------------
        // LOOP
        // -----------------------------
        while (opModeIsActive()) {
            moveDrvTrain(0,0,0);
        }
    }
}
