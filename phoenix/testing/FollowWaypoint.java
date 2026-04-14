package org.firstinspires.ftc.teamcode.phoenix;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.phoenix.geometry.Pose;
import org.firstinspires.ftc.teamcode.phoenix.localization.constants.TwoWheelConstants;
import org.firstinspires.ftc.teamcode.phoenix.localization.localizers.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.phoenix.util.Turret;

@TeleOp(name = "FollowWaypoint", group = "Drive")
public class FollowWaypoint extends LinearOpMode {

    // --- Hardware Declarations ---
    private DcMotorEx intakeDrive;
    private DcMotor backLeftMotor, frontLeftMotor, frontRightMotor, backRightMotor;
    private DcMotorEx shoot = null, shootstraff = null;
    private IMU imu;

    // --- Localization ---
    private TwoWheelConstants twoWheelConstants;
    private TwoWheelLocalizer twoWheelLocalizer;
    private int rawX, rawY;

    // --- Variables & Timers ---
    private ElapsedTime timer = new ElapsedTime();
    private boolean wasTranslating = false;

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

    // ===== PURE PURSUIT PARAMS =====
    double lookaheadDist = 50; // cm (15–30 là đẹp)
    double kP_drive = 0.025;
    double kP_heading = 1;
    
    double maxVel = 1;
    double maxTurn = 0.5;
    
    double endSlowRadius = 10; // cm
    double endTolerance = 1.5;

    public static class Waypoint {
        public double x, y, heading;
    
        public Waypoint(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }


    // =========================================================================
    // HELPER METHODS
    // =========================================================================

    private double angleWrap(double rad) {
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        while (rad < -Math.PI) rad += 2.0 * Math.PI;
        return rad;
    }

    /**
     * Hàm điều khiển Chassis Mecanum với Heading Hold (Giữ hướng)
     */
    public void moveDrvTrain(double y, double x, double z) {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 0.02;

        // Deadband để tránh noise làm hỏng heading hold
        if (Math.abs(x) < moveDeadband) x = 0;
        if (Math.abs(y) < moveDeadband) y = 0;
        if (Math.abs(z) < turnDeadband) z = 0;

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
        double rotX = y * Math.cos(-botHeading) - x * Math.sin(-botHeading);
        double rotY = y * Math.sin(-botHeading) + x * Math.cos(-botHeading);
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
        rawX = -backRightMotor.getCurrentPosition();
        
        // Sân xanh logic
        twoWheelLocalizer.update();
        Pose pose = twoWheelLocalizer.getPose();
        double angle_unit = Math.atan2(135 - pose.getY(), 10 - pose.getX()) - pose.getHeading();
        
        while (angle_unit > Math.PI) angle_unit -= 2 * Math.PI;
        while (angle_unit <= -Math.PI) angle_unit += 2 * Math.PI;
        

        telemetry.addData("y", pose.getY());
        telemetry.addData("x", pose.getX());
        telemetry.addData("angle unit", Math.toDegrees(angle_unit));
        telemetry.addData("atan2", Math.atan2(135 - pose.getY(), 10 - pose.getX()));
        telemetry.addData("Heading rad", botHeading);
        telemetry.addData("Heading deg", (Math.toDegrees(botHeading) + 360) % 360);
        telemetry.addData("raw_y", rawY);
        telemetry.addData("raw_x", rawX);
        telemetry.addData("Turning", isTurning);
        telemetry.addData("turnCmd", turnCmd);
        telemetry.update();
    }

    public Waypoint findLookahead(
            Pose pose,
            Waypoint p1,
            Waypoint p2,
            double lookahead
    ) {
        double cx = pose.getX();
        double cy = pose.getY();
    
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
    
        double fx = p1.x - cx;
        double fy = p1.y - cy;
    
        double a = dx * dx + dy * dy;
        double b = 2 * (fx * dx + fy * dy);
        double c = (fx * fx + fy * fy) - lookahead * lookahead;
    
        double disc = b * b - 4 * a * c;
        if (disc < 0) return null;
    
        disc = Math.sqrt(disc);
    
        double t1 = (-b - disc) / (2 * a);
        double t2 = (-b + disc) / (2 * a);
    
        double t = -1;
        if (t2 >= 0 && t2 <= 1) t = t2;
        else if (t1 >= 0 && t1 <= 1) t = t1;
    
        if (t < 0) return null;
    
        double lx = p1.x + dx * t;
        double ly = p1.y + dy * t;
    
        double heading = Math.atan2(dy, dx);
        return new Waypoint(lx, ly, heading);
    }
    int currentSegment = 0;
    
    public boolean purePursuitFollow(Waypoint[] path) {
    
        twoWheelLocalizer.update();
        Pose pose = twoWheelLocalizer.getPose();
    
        Waypoint lookahead = null;
    
        // tìm lookahead trên path
        for (int i = currentSegment; i < path.length - 1; i++) {
            Waypoint p1 = path[i];
            Waypoint p2 = path[i + 1];
    
            Waypoint la = findLookahead(pose, p1, p2, lookaheadDist);
            if (la != null) {
                lookahead = la;
                currentSegment = i;
                break;
            }
        }
    
        // fallback: nhìn thẳng tới điểm cuối
        if (lookahead == null) {
            Waypoint last = path[path.length - 1];
            lookahead = new Waypoint(last.x, last.y, last.heading);
        }
    
        // vector tới lookahead (FIELD)
        double errX = lookahead.x - pose.getX();
        double errY = lookahead.y - pose.getY();
        double dist = Math.hypot(errX, errY);
    
        // vận tốc tịnh tiến
        double vx = kP_drive * errX;
        double vy = kP_drive * errY;
    
        double mag = Math.hypot(vx, vy);
        if (mag > maxVel) {
            vx = vx / mag * maxVel;
            vy = vy / mag * maxVel;
        }
    
        // slowdown gần cuối path
        Waypoint end = path[path.length - 1];
        double endDist = Math.hypot(end.x - pose.getX(), end.y - pose.getY());
    
        if (endDist < endSlowRadius) {
            double scale = Math.max(0.25, endDist / endSlowRadius);
            vx *= scale;
            vy *= scale;
        }
    
        // heading control
        double headingErr = angleWrap(lookahead.heading - pose.getHeading());
        double omega = Range.clip(kP_heading * headingErr, -maxTurn, maxTurn);
        // double omega = 0;
        moveDrvTrain(-vy, vx, omega);
    
        // DONE condition
        if (endDist < endTolerance &&
                Math.abs(angleWrap(end.heading - pose.getHeading())) < Math.toRadians(3)) {
            moveDrvTrain(0, 0, 0);
            return true;
        }
    
        return false;
    }
    
    

    // =========================================================================
    // MAIN OP MODE
    // =========================================================================
    @Override
    public void runOpMode() throws InterruptedException {

        // -----------------------------
        // 0) HARDWARE MAP
        // -----------------------------
        // twoWheelConstants = new TwoWheelConstants();
        // twoWheelLocalizer = new TwoWheelLocalizer(hardwareMap, twoWheelConstants, new Pose(0, 0, Math.toRadians(0)));

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
        twoWheelConstants = new TwoWheelConstants();
        twoWheelLocalizer = new TwoWheelLocalizer(hardwareMap, twoWheelConstants, new Pose(0, 0, Math.toRadians(0)));
        waitForStart();
        while(!gamepad1.a && opModeIsActive()) {
            moveDrvTrain(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
            telemetry.addLine("Press A to start");
            telemetry.update();
        }
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        twoWheelConstants = new TwoWheelConstants();
        twoWheelLocalizer = new TwoWheelLocalizer(hardwareMap, twoWheelConstants, new Pose(0, 0, Math.toRadians(0)));

        // -----------------------------
        // START OF AUTONOMOUS
        // -----------------------------
        


        // Đồng bộ targetHeading ngay sau resetYaw
        targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        lastError = 0;
        integral = 0;
        wasTranslating = false;

        if (isStopRequested()) return;

        timer.reset();

        currentSegment = 0;

        Waypoint[] path = new Waypoint[] {
            new Waypoint(0, 0, Math.toRadians(0)),
            new Waypoint(20, 20, Math.toRadians(0)),
            new Waypoint(40, 40, Math.toRadians(0)),
            new Waypoint(80, 80, Math.toRadians(0))
        };

        while (opModeIsActive()) {
            // moveDrvTrain(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
            purePursuitFollow(path);

        }
    }
}