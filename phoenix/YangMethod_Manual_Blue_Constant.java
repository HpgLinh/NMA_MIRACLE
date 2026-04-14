package org.firstinspires.ftc.teamcode.phoenix;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.phoenix.util.Turret;
import org.firstinspires.ftc.teamcode.phoenix.util.Toggler;

import org.firstinspires.ftc.teamcode.phoenix.localization.localizers.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.phoenix.localization.constants.TwoWheelConstants;
import org.firstinspires.ftc.teamcode.phoenix.geometry.Pose;

import android.content.SharedPreferences;

@TeleOp(name = "HPL MANUAL BLUE", group = "Drive")
public class YangMethod_Manual_Blue_Constant extends LinearOpMode {
    public Turret turret;
    
    IMU imu;
    DcMotor backLeftMotor;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    
    TwoWheelConstants twoWheelConstants;
    TwoWheelLocalizer twoWheelLocalizer;
    
    double botHeading;
    
    
    double P = 30;
    double I = 0.01;
    double D = 0;
    double F = 15;
    // ---------- Heading Hold PID ----------
    // Bạn có thể tune lại 3 giá trị này
    double kp = 1;      // vì error đang dùng RAD -> kp thường lớn hơn bản DEG
    double ki = 0.0;
    double kd = 0.08;

    double targetHeading = 0.0;  // rad
    double lastError = 0.0;
    double integral = 0.0;
    
    double turret_offset = 0, angle_unit = 0;

    private DcMotorEx shoot = null, shootstraff = null;

    private Servo srvOval = null;

    boolean wasTranslating = false;

    // Deadbands
    
    double y ;
    double x ;
    double rx;

    Pose pose;
    
    final double moveDeadband = 0.05;
    final double turnDeadband = 0.10;

    // Toggler
    Toggler togglerStateIntakeIn = new Toggler();
    Toggler togglerStateIntakeOut = new Toggler();
    Toggler togglerShootSpeedIncrease = new Toggler();
    Toggler togglerShootSpeedDecrease = new Toggler();
    Toggler togglerOvalIncrease = new Toggler();
    Toggler togglerOvalDecrease = new Toggler();
    Toggler toggler2dpadright = new Toggler();
    Toggler toggler2dpadleft = new Toggler();
    Toggler toggler2dpadup = new Toggler();
    Toggler toggler2dpaddown = new Toggler();


    double lloffset = 0;

    ElapsedTime timer = new ElapsedTime(), counting = new ElapsedTime();


    private double angleWrap(double rad) {
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        while (rad < -Math.PI) rad += 2.0 * Math.PI;
        return rad;
    }


    // -----------------------------
    private double clip01(double v) {
        return Range.clip(v, 0.0, 1.0);
    }
    private int get_power(double distance){
        return (int)Math.round(199881e-8 * Math.pow(distance, 3) - 0.000105039 * Math.pow(distance, 2) + 0.113276 * distance + 1906.1956);
    }
    private double get_oval(double distance) {
        return 6.28615e-12 * Math.pow(distance, 3) - 6.25259e-8 * Math.pow(distance, 2) + 0.000190833 * distance - 0.138137;
    }

    void readGamepad() {
        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x;
    }

    void moveDrvTrain() {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 0.02;

        // -----------------------------
        // 2) INPUT
        // -----------------------------

        // Deadband để tránh noise làm hỏng heading hold
        if (Math.abs(x) < moveDeadband) x = 0;
        if (Math.abs(y) < moveDeadband) y = 0;
        if (Math.abs(rx) < turnDeadband) rx = 0;

        // Reset heading khi bấm options
        if (gamepad1.options && gamepad1.share) {
            imu.resetYaw();
            targetHeading = 0.0;
            lastError = 0;
            integral = 0;

            while (gamepad1.options && opModeIsActive() && gamepad1.share) {
                ; // do nothing
            }
        }

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        boolean isTurning = (rx != 0);
        boolean isTranslating = (Math.hypot(x, y) > 0);

        // -----------------------------
        // 3) ANTI-DRIFT CAPTURE
        // Khi vừa bắt đầu tịnh tiến mà không xoay
        // -----------------------------
        if (isTranslating && !wasTranslating && !isTurning) {
            targetHeading = botHeading;
            lastError = 0;
            integral = 0;
        }
        wasTranslating = isTranslating;

        // -----------------------------
        // 4) HEADING HOLD OUTPUT
        // -----------------------------
        double kpMove = kp;
        double kiMove = ki;
        double kdMove = kd;

        double kpStill = 0.5;      // bạn có thể thử 0.25 -> 0.5
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
            //if(Math.abs(error) < 1) break;
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
        // 5) FIELD CENTRIC
        // -----------------------------
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX *= 1.1; // Counteract imperfect strafing

        // -----------------------------
        // 6) MECANUM POWER
        // -----------------------------
        double turnWeight = (isTranslating && !isTurning) ? 0.35 : 1.0;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turnCmd) * turnWeight, 1);
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
        
        twoWheelLocalizer.update();
        pose = twoWheelLocalizer.getPose();
    }


    @Override
    public void runOpMode() throws InterruptedException {
        SharedPreferences prefs =
                hardwareMap.appContext.getSharedPreferences("FTC_DATA", 0);
        double loadX = prefs.getFloat("x", 0f);
        double loadY = prefs.getFloat("y", 0f);
        double loadHeading = prefs.getFloat("heading", 0f);
        double turretState = prefs.getFloat("turret degree", 0f);
        // -----------------------------
        // 0) HARDWARE MAP
        // -----------------------------
        twoWheelConstants = new TwoWheelConstants();
        twoWheelLocalizer = new TwoWheelLocalizer(hardwareMap, twoWheelConstants, new Pose(loadX, loadY, loadHeading));
        
        Limelight3A limelight;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(9);
        
        turret = new Turret(hardwareMap, turretState);
        backLeftMotor  = hardwareMap.dcMotor.get("drv3");
        frontLeftMotor = hardwareMap.dcMotor.get("drv2");
        frontRightMotor = hardwareMap.dcMotor.get("drv1");
        backRightMotor = hardwareMap.dcMotor.get("drv4");

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        DcMotorEx intakeDrive = hardwareMap.get(DcMotorEx.class, "drvintake");
        intakeDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        srvOval = hardwareMap.get(Servo.class, "srvOval0");
        Servo srvIntake = hardwareMap.get(Servo.class, "srvIntake");
        Servo srvLoad = hardwareMap.get(Servo.class, "srvLoad");
        CRServo srvblock0 = hardwareMap.get(CRServo.class, "srvblock0");
        CRServo srvblock1 = hardwareMap.get(CRServo.class, "srvblock1");
        
        shoot = hardwareMap.get(DcMotorEx.class, "drvShoot");
        shootstraff = hardwareMap.get(DcMotorEx.class, "drvShootstraff");
        
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
        int power = 0;
        int powerForward = 2350;
        int powerReverse = 2600;

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

        sleep(300);
        waitForStart();

        limelight.start();
        // Reset yaw theo thói quen code gốc
        imu.resetYaw();

        // Servo init
        srvblock1.setPower(-1.0);
        srvblock0.setPower(-1.0);
        srvIntake.setPosition(0.25);
        srvLoad.setPosition(0.0);
        srvOval.setPosition(0);
        double target_degree = 0;

        // Đồng bộ targetHeading ngay sau resetYaw
        targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        lastError = 0;
        integral = 0;
        boolean prevX=false, prevB=false, prevLB=false, prevRB=false;
        boolean prevOpt=false, prevShare = false;
        boolean prevDpadUp=false, prevDpadDown=false, prevDpadLeft=false, prevDpadRight=false;
        boolean prevRT=false, prevLT=false;
        boolean prevA=false, prevY=false;
        boolean intakeDeployed = false; // false -> stow
        boolean loadOpen = false;
        wasTranslating = false;
        boolean head_lock = false, close_tag = false;
        
        
        int power_headlock_up = 1800;
        int power_headlock_down = 2300;
        double headlock_offset = 0;
        int time_lock = 0;
        
        if (isStopRequested()) return;
        
    
        timer.reset();
        
        boolean toggle_shooting = true;
        int offsetShoot = 0;
        // -----------------------------
        // LOOP
        // -----------------------------
        while (opModeIsActive()) {
            shoot.setVelocity(toggle_shooting ? power : 0);
            shootstraff.setVelocity(toggle_shooting ? power : 0);
            
            readGamepad();
            moveDrvTrain();
            
            if(toggler2dpadright.shouldToggle(gamepad2.dpad_right)) headlock_offset -= 1;
            if(toggler2dpadleft.shouldToggle(gamepad2.dpad_left)) headlock_offset += 1;
            
            if(toggler2dpaddown.shouldToggle(gamepad2.left_trigger > 0.9)) headlock_offset += 5;
            if(toggler2dpadup.shouldToggle(gamepad2.right_trigger > 0.9)) headlock_offset -= 5;
            
            if(gamepad1.leftBumperWasPressed()){
                head_lock ^= true;
            }
            
            if(head_lock) {
                angle_unit = Math.atan2(138-pose.getY(),10-pose.getX()) - pose.getHeading();
                while (angle_unit > Math.PI) angle_unit -= 2 * Math.PI;
                while (angle_unit <= -Math.PI) angle_unit += 2 * Math.PI;
                turret.setPosition(Math.toDegrees(angle_unit) + turret_offset - lloffset);
                power = close_tag ? power_headlock_up + (angleWrap(botHeading) < Math.PI/2 && angleWrap(botHeading) > -Math.PI/2 ? 0 : 150) : power_headlock_down;
            } else if(!close_tag){
                if(angleWrap(botHeading) < Math.PI/2 && angleWrap(botHeading) > -Math.PI/2) 
                {
                    turret.setPosition(-66 + turret_offset);//heading lúc khởi động
                    srvOval.setPosition(0.14);
                    power = powerForward;
                }
                else
                {
                    turret.setPosition(115 + turret_offset);//heading quay ngược về sau (để intake phía sau)
                    srvOval.setPosition(0.16);
                    power = powerReverse;
                }
            } else {
                turret.setPosition(40);
                srvOval.setPosition(0.06);
                power = 1800;
            }
            
            if(gamepad1.options) toggle_shooting ^= true;
            
            // shoot.setVelocity(toggle_shooting ? power : 0);
            // shootstraff.setVelocity(toggle_shooting ? power : 0);

            // -----------------------------
            // 7) INTAKE LOGIC (GIỮ NGUYÊN)
            // -----------------------------
            
            // if (gamepad1.a) {
            //     while (gamepad1.a) ;
            //     stateIntake = (stateIntake != 0) ? 0 : 1;
            // }
            if (togglerStateIntakeIn.shouldToggle(gamepad1.a)) {
                stateIntake = (stateIntake != 0) ? 0 : 1;
            }
            
            // if (gamepad1.y) {
            //     while (gamepad1.y) ;
            //     stateIntake = (stateIntake != 0) ? 0 : -1;
            // }
            if (togglerStateIntakeOut.shouldToggle(gamepad1.y)) {
                stateIntake = (stateIntake != 0) ? 0 : -1;
            }


        
            boolean rtPress = (gamepad2.y);
            // if (rtPress && !prevRT) 
            // {
            //     if(angleWrap(botHeading) < Math.PI/2 && angleWrap(botHeading) > -Math.PI/2) 
            //     {
            //         powerForward += 50;
            //     }
            //     else
            //     {
            //         powerReverse += 50;
            //     }
            // }
            // prevRT = rtPress;
            if (togglerShootSpeedIncrease.shouldToggle(rtPress)) {
                if(angleWrap(botHeading) < Math.PI/2 && angleWrap(botHeading) > -Math.PI/2) 
                {
                    powerForward += 50;
                }
                else
                {
                    powerReverse += 50;
                }
                if(head_lock && close_tag) power_headlock_up += 50;
                if(head_lock && !close_tag) power_headlock_down += 50;
            }

            boolean ltPress = (gamepad2.a);
            // if (ltPress && !prevLT) 
            // {
            //     if(angleWrap(botHeading) < Math.PI/2 && angleWrap(botHeading) > -Math.PI/2) 
            //     {
            //         powerForward -= 50;
            //     }
            //     else
            //     {
            //         powerReverse -= 50;
            //     }
            // }
            // prevLT = ltPress;
            if (togglerShootSpeedDecrease.shouldToggle(ltPress)) {
                if(angleWrap(botHeading) < Math.PI/2 && angleWrap(botHeading) > -Math.PI/2) 
                {
                    powerForward -= 50;
                }
                else
                {
                    powerReverse -= 50;
                }
                if(head_lock && !close_tag) power_headlock_down -= 50;
                if(head_lock && close_tag) power_headlock_up -= 50;
            }

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
               telemetry.addData("Tx", result.getTx());
               telemetry.addData("Ty", result.getTy());
            }
            // boolean lb = gamepad1.left_bumper;
            // if(lb && !prevLB) srvIntake.setPosition(0);
            // if(!lb && prevLB) srvIntake.setPosition(0.25);
            // prevLB = lb;
        
            boolean rb = gamepad1.right_bumper;
            if (rb && !prevRB) {// vừa nhấn
                if(close_tag){
                srvLoad.setPosition(0.25);
                loadOpen = true;}
                if (result != null && result.isValid()) {
                   lloffset += result.getTx();
                } else {
                    lloffset += 0;
                }
                counting.reset();
            }
            if(rb && counting.seconds() >= 0.001 && !close_tag) {
                time_lock++;
                counting.reset();
            }
            if(time_lock >= 10 && !close_tag){
                srvLoad.setPosition(0.25);
                loadOpen = true;
                time_lock = -500000;
            }
            if (!rb && prevRB) {
                time_lock = 0;
                srvLoad.setPosition(0.1);
                loadOpen = false;
            }
            prevRB = rb;

            if (stateIntake == 0) intakeDrive.setVelocity(0);
            else if (stateIntake == -1) intakeDrive.setVelocity(-4000);
            else if (stateIntake == 1) intakeDrive.setVelocity(loadOpen ? 2300 : 4000);


            // if (gamepad1.left_bumper) {
            //     while (gamepad1.left_bumper) ;
            //     srvIntake.setPosition(0.12 + (0.5 - srvIntake.getPosition()));
            // }

            boolean dUp = gamepad1.dpad_up;
            // if (dUp && !prevDpadUp) srvOval.setPosition(clip01(Math.min(0.4, srvOval.getPosition() + 0.04)));
            // prevDpadUp = dUp;
            if (togglerOvalIncrease.shouldToggle(dUp)) {
                srvOval.setPosition(clip01(Math.min(0.4, srvOval.getPosition() + 0.02)));
            }


            boolean dDown = gamepad1.dpad_down;
            // if (dDown && !prevDpadDown) srvOval.setPosition(clip01(Math.max(0.0, srvOval.getPosition() - 0.04)));
            // prevDpadDown = dDown;
            if (togglerOvalDecrease.shouldToggle(dDown)) {
                srvOval.setPosition(clip01(Math.max(0.0, srvOval.getPosition() - 0.02)));
            }


            // boolean sharebutton = gamepad1.share;
            // if (sharebutton && !prevShare) {
            //     imu.initialize(parameters);
            //     imu.resetYaw();
            //     targetHeading = 0.0;
            //     lastError = 0;
            //     integral = 0;
            //     wasTranslating = false;
            // }
            // prevShare = sharebutton;
            
            if(gamepad1.x) {
                close_tag = false;
                srvOval.setPosition(0.12);
            }

            if(gamepad1.b) {
                close_tag = true;
                srvOval.setPosition(0.08);
            }
           // -----------------------------
            // 8) TELEMETRY
            // -----------------------------
            telemetry.addData("time lock", time_lock);
            telemetry.addLine("=========== AIMING SYSTEM =============");
            telemetry.addData("head_lock", head_lock);
            telemetry.addData("y", pose.getY());
            telemetry.addData("x", pose.getX());
            telemetry.addData("Turret Heading", turret.currentDegree);
            telemetry.addData("Turret offset", turret_offset);
            telemetry.addData("Goal degree", Math.toDegrees(angle_unit) + turret_offset - lloffset);
            
            telemetry.addLine("=========== FLYWHEEL SYSTEM =============");
            telemetry.addData("Oval", srvOval.getPosition());
            telemetry.addData("shoot velocity", shoot.getVelocity());
            telemetry.addData("shootstraff velocity", shootstraff.getVelocity());
            telemetry.addData("shoot encoder", shoot.getCurrentPosition());
            telemetry.addData("shootStraff encoder", shootstraff.getCurrentPosition());
            telemetry.addData("powerForward", powerForward);
            telemetry.addData("powerReverse", powerReverse);
            telemetry.addData("power", power);
            
            telemetry.addLine("=========== MECHANISM SYSTEM =============");
            telemetry.addData("State", stateIntake);
            telemetry.addData("Heading deg", Math.toDegrees(angleWrap(botHeading)));
            telemetry.addData("Target deg", Math.toDegrees(targetHeading));
            telemetry.addData("loadX", loadX);
            telemetry.addData("loadY", loadY);
            telemetry.addData("loadHeading", loadHeading);

            telemetry.update();
        }
    }
}
