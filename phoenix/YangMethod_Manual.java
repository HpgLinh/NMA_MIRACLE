package org.firstinspires.ftc.teamcode.phoenix;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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

@TeleOp(name = "Dương's Method", group = "Drive")
public class YangMethod_Manual extends LinearOpMode {

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

    private DcMotorEx shoot = null, shootstraff = null;

    private Servo srvOval = null;

    boolean wasTranslating = false;

    // Deadbands
    final double moveDeadband = 0.05;
    final double turnDeadband = 0.10;

    private double angleWrap(double rad) {
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        while (rad < -Math.PI) rad += 2.0 * Math.PI;
        return rad;
    }

    // -----------------------------
    private double clip01(double v) {
        return Range.clip(v, 0.0, 1.0);
    }
    
    void moveDriveTrain(double x, double y, double z)
    {
    }
    
    @Override
    public void runOpMode() throws InterruptedException {

        // -----------------------------
        // 0) HARDWARE MAP (MAP CÁI ĐỊT CON MẸ, CODE GPT ĂN ĐẦU BUỒI, ĂN CỨT)
        // -----------------------------
        TwoWheelConstants twoWheelConstants = new TwoWheelConstants();
        TwoWheelLocalizer twoWheelLocalizer = new TwoWheelLocalizer(hardwareMap, twoWheelConstants, new Pose(0,0,0));

        DcMotor backLeftMotor  = hardwareMap.dcMotor.get("drv3");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("drv2");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("drv1");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("drv4");

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


        Servo srvTurret1 = hardwareMap.get(Servo.class, "srvTurret0");
        Servo srvTurret2 = hardwareMap.get(Servo.class, "srvTurret1");
        DcMotor intakeDrive = hardwareMap.get(DcMotor.class, "drvintake");
        srvOval = hardwareMap.get(Servo.class, "srvOval0");
        Servo srvIntake = hardwareMap.get(Servo.class, "srvIntake");
        Servo srvLoad = hardwareMap.get(Servo.class, "srvLoad");
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
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);

        sleep(300);
        waitForStart();

        // Reset yaw theo thói quen code gốc (THÓI QUEN LOZ NÀO, TÍNH BẮT BÀI TAO À, TUỔI LOZ)
        imu.resetYaw();

        // Servo init
        srvIntake.setPosition(0.25);
        srvTurret2.setPosition(0.0);
        srvTurret1.setPosition(0.0);
        srvLoad.setPosition(0.0);
        srvOval.setPosition(0);
        srvblock0.setPower(-1);
        srvblock1.setPower(-1);

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

        ElapsedTime timer = new ElapsedTime();
        timer.reset();


        boolean toggle_shooting = true;
        // -----------------------------
        // LOOP
        // -----------------------------
        while (opModeIsActive()) {
            
            shoot.setVelocity(toggle_shooting ? power : 0);
            shootstraff.setVelocity(toggle_shooting ? power : 0);


            double dt = timer.seconds();
            timer.reset();
            if (dt <= 0) dt = 0.02;

            // -----------------------------
            // 2) INPUT
            // -----------------------------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Deadband để tránh noise làm hỏng heading hold (NHƯNG BỐ MÀY THÍCH HỎNG, Ý KIẾN?)
            if (Math.abs(x) < moveDeadband) x = 0;
            if (Math.abs(y) < moveDeadband) y = 0;
            if (Math.abs(rx) < turnDeadband) rx = 0;

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

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            boolean isTurning = (rx != 0);
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
                turnCmd = rx;

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
                    Math.abs(rotY) + Math.abs(rotX) + Math.abs(turnCmd) * turnWeight,
                    1
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

            // -----------------------------
            // 7) INTAKE LOGIC (GIỮ NGUYÊN) (??? MÀY HỌC ĐÂU CÁI THÓI MỞ NGOẶC CHỨA QUOTE ĐẤY? BỐ MÀY CHƯA DẠY MÀY BÀI NÀY NHÉ)
            // -----------------------------
            if (gamepad1.x) {
                while (gamepad1.x) ;
                stateIntake = (stateIntake != 0) ? 0 : 1;
            }

            if (gamepad1.b) {
                while (gamepad1.b) ;
                stateIntake = (stateIntake != 0) ? 0 : -1;
            }



            boolean rtPress = (gamepad1.right_trigger > 0.9);
            if (rtPress && !prevRT) power += 50;
            prevRT = rtPress;

            boolean ltPress = (gamepad1.left_trigger > 0.9);
            if (ltPress && !prevLT) power -= 50;
            prevLT = ltPress;

            boolean lb = gamepad1.left_bumper;
            if(lb && !prevLB) srvIntake.setPosition(0);
            if(!lb && prevLB) srvIntake.setPosition(0.25);
            prevLB = lb;



            boolean rb = gamepad1.right_bumper;
            if (rb && !prevRB) {          // vừa nhấn (??)
                srvLoad.setPosition(0.25);
                loadOpen = true;
            }
            if (!rb && prevRB) {          // vừa thả (??????????)
                srvLoad.setPosition(0.1);
                loadOpen = false;
            }
            prevRB = rb;

            if (stateIntake == 0) intakeDrive.setPower(0);
            else if (stateIntake == -1) intakeDrive.setPower(-1);
            else if (stateIntake == 1) intakeDrive.setPower(loadOpen ? 0.6 : 1);


            if (gamepad1.left_bumper) {
                while (gamepad1.left_bumper) ;
                srvIntake.setPosition(0.12 + (0.5 - srvIntake.getPosition()));
            }


            if(gamepad1.dpadRightWasPressed()) {srvTurret1.setPosition(srvTurret1.getPosition() + 0.02); srvTurret2.setPosition(srvTurret2.getPosition() + 0.02);}
            if(gamepad1.dpadLeftWasPressed()) {srvTurret1.setPosition(srvTurret1.getPosition() - 0.02); srvTurret2.setPosition(srvTurret2.getPosition() - 0.02);}

            boolean dUp = gamepad1.dpad_up;
            if (dUp && !prevDpadUp) srvOval.setPosition(clip01(Math.min(0.4, srvOval.getPosition() + 0.04)));
            prevDpadUp = dUp;

            boolean dDown = gamepad1.dpad_down;
            if (dDown && !prevDpadDown) srvOval.setPosition(clip01(Math.max(0.0, srvOval.getPosition() - 0.04)));
            prevDpadDown = dDown;

            if(gamepad1.options) toggle_shooting ^= true;

            boolean sharebutton = gamepad1.share;
            if (sharebutton && !prevShare) {
                imu.initialize(parameters);
                imu.resetYaw();
                targetHeading = 0.0;
                lastError = 0;
                integral = 0;
                wasTranslating = false;
            }
            prevShare = sharebutton;


            if(gamepad1.a) {
                power = 1200;
                srvOval.setPosition(0.16);
                srvTurret2.setPosition(-0.251 + 0.5);
                srvTurret1.setPosition(-0.251 + 0.5);
            }

            if(gamepad1.y) {
                power = 850;
                srvOval.setPosition(0.0);
                srvTurret2.setPosition(-0.15 + 0.5);
                srvTurret1.setPosition(-0.15 + 0.5);
            }
     
            // -----------------------------
            // 8) TELEMETRY (MỘT LÀ MÀY TELE GẤP VỀ BỆ ĐÁ CỔ, HAI LÀ TAO VẶN CỔ MÀY)
            // -----------------------------
            
            twoWheelLocalizer.update();
            Pose pose = twoWheelLocalizer.getPose();
            
            telemetry.addData("y", pose.getY());
            telemetry.addData("x", pose.getX());
            telemetry.addData("Heading rad", botHeading);
            telemetry.addData("Heading deg", (Math.toDegrees(botHeading) + 360) % 360);
            telemetry.addData("raw_y",(frontLeftMotor.getCurrentPosition()));
            telemetry.addData("raw_x",(backRightMotor.getCurrentPosition()));
            telemetry.addData("turret motor", shoot.getVelocity());
            telemetry.addData("Intake_servo", srvIntake.getPosition());
            telemetry.addData("Oval", srvOval.getPosition());
            telemetry.addData("turret", srvTurret1.getPosition());
            telemetry.addData("power", power);
            telemetry.addData("State", stateIntake);
            telemetry.addData("Target deg", Math.toDegrees(targetHeading));
            telemetry.addData("Turning", isTurning);
            telemetry.addData("turnCmd", turnCmd);
            telemetry.update();
        }
    }
}
