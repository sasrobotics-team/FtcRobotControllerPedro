package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "AutoInitial2V2_627", group = "Auto")
public class AutoInitialV2_2627 extends LinearOpMode {

    // ── HARDWARE ──────────────────────────────────────────────────────────────
    private DcMotorEx front_left, front_right, back_left, back_right;
    private DcMotorEx shooter, intake;
    private Servo shooter_servo;
    private IMU imu;

    // ── DRIVE CONSTANTS (CHECK THESE AGAINST YOUR ROBOT) ──────────────────────
    // Encoder ticks per motor output revolution.
    //   goBILDA 5203 312 RPM = 537.7 | 435 RPM = 384.5 | REV HD Hex 20:1 = 560
    static final double TICKS_PER_REV     = 537.7;
    static final double WHEEL_DIAMETER_IN = 3.78;   // goBILDA 96 mm mecanum
    static final double GEAR_RATIO        = 1.0;    // external gearing, wheel/motor
    static final double TICKS_PER_INCH    =
            (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER_IN * Math.PI);

    // Mecanum wheels slip a bit sideways; bump this until strafes are accurate.
    static final double STRAFE_CORRECTION = 1.10;

    // ── TURN CONSTANTS ────────────────────────────────────────────────────────
    static final double TURN_KP               = 0.02;  // power per degree of error
    static final double MIN_TURN_POWER        = 0.08;  // enough to overcome friction
    static final double HEADING_TOLERANCE_DEG = 1.5;

    // ── MECHANISM CONSTANTS ───────────────────────────────────────────────────
    static final double SHOOTER_VELOCITY  = 1500;  // ticks/sec
    static final double SHOOTER_TOLERANCE = 50;    // ticks/sec "close enough"
    static final double INTAKE_VELOCITY   = 1000;  // ticks/sec

    static final double SERVO_REST = 0.0;  // tune on the robot
    static final double SERVO_FIRE = 0.5;  // tune on the robot

    private final ElapsedTime timer = new ElapsedTime();

    // ══════════════════════════════════════════════════════════════════════════
    @Override
    public void runOpMode() {
        initHardware();

        // Init loop: shows heading so you can confirm the IMU before starting.
        while (opModeInInit()) {
            telemetry.addData("Status", "Initialized — press START");
            telemetry.addData("Heading", "%.1f°", getHeading());
            telemetry.update();
        }
        if (isStopRequested()) return;

        imu.resetYaw();  // "0°" = the way the robot faces at the start

        // ── THE ROUTINE ───────────────────────────────────────────────────────
        // Example only: replace with your actual path.
        driveInches(24, 0.5);          // forward 24 in
        turnToHeading(45, 0.4);        // face 45° left
        shoot(3);                      // fire 3 game pieces
        turnToHeading(0, 0.4);         // face forward again
        setIntake(INTAKE_VELOCITY);    // run intake while driving
        driveInches(18, 0.3);
        setIntake(0);
        strafeInches(-12, 0.5);        // strafe 12 in left (park)

        stopDrive();
    }

    // ══════════════════════════════════════════════════════════════════════════
    // SETUP
    // ══════════════════════════════════════════════════════════════════════════
    private void initHardware() {
        front_left  = hardwareMap.get(DcMotorEx.class, "front_left");
        front_right = hardwareMap.get(DcMotorEx.class, "front_right");
        back_left   = hardwareMap.get(DcMotorEx.class, "back_left");
        back_right  = hardwareMap.get(DcMotorEx.class, "back_right");
        intake     = hardwareMap.get(DcMotorEx.class, "intake");
        shooter    = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter_servo = hardwareMap.get(Servo.class, "shooter_servo");

        // Same directions as your TeleOp. FIRST TEST: driveInches(12, 0.3)
        // must go FORWARD. If it goes backward, flip all four drive directions.
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{front_left, front_right, back_left, back_right, intake}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        shooter_servo.setPosition(SERVO_REST);
    }

    // ══════════════════════════════════════════════════════════════════════════
    // DRIVE HELPERS
    // ══════════════════════════════════════════════════════════════════════════

    /** Positive = forward, negative = backward. */
    private void driveInches(double inches, double power) {
        int t = (int) Math.round(inches * TICKS_PER_INCH);
        moveByTicks(t, t, t, t, power, timeoutFor(inches, power));
    }

    /** Positive = right, negative = left. Same wheel pattern as your TeleOp x-axis. */
    private void strafeInches(double inches, double power) {
        int t = (int) Math.round(inches * TICKS_PER_INCH * STRAFE_CORRECTION);
        moveByTicks(t, -t, -t, t, power, timeoutFor(inches, power));
    }

    /** Runs all four wheels to relative encoder targets, with a safety timeout. */
    private void moveByTicks(int fl, int fr, int bl, int br, double power, double timeoutS) {
        // Target must be set BEFORE switching to RUN_TO_POSITION.
        front_left.setTargetPosition(front_left.getCurrentPosition() + fl);
        front_right.setTargetPosition(front_right.getCurrentPosition() + fr);
        back_left.setTargetPosition(back_left.getCurrentPosition() + bl);
        back_right.setTargetPosition(back_right.getCurrentPosition() + br);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(Math.abs(power));

        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutS
                && front_left.isBusy() && front_right.isBusy()
                && back_left.isBusy() && back_right.isBusy()) {
            telemetry.addData("Action", "Driving");
            telemetry.addData("FL tgt/pos", "%d / %d",
                    front_left.getTargetPosition(), front_left.getCurrentPosition());
            telemetry.addData("FR tgt/pos", "%d / %d",
                    front_right.getTargetPosition(), front_right.getCurrentPosition());
            telemetry.addData("Heading", "%.1f°", getHeading());
            telemetry.update();
        }

        stopDrive();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Turns to an absolute heading (degrees). 0 = start direction,
     * positive = counter-clockwise (left), negative = clockwise (right).
     */
    private void turnToHeading(double targetDeg, double maxPower) {
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        timer.reset();

        while (opModeIsActive() && timer.seconds() < 3.0) {
            double error = AngleUnit.normalizeDegrees(targetDeg - getHeading());
            if (Math.abs(error) < HEADING_TOLERANCE_DEG) break;

            double turn = Range.clip(error * TURN_KP, -maxPower, maxPower);
            if (Math.abs(turn) < MIN_TURN_POWER) turn = Math.copySign(MIN_TURN_POWER, turn);

            // Positive turn → left side backward, right side forward → CCW.
            front_left.setPower(-turn);
            back_left.setPower(-turn);
            front_right.setPower(turn);
            back_right.setPower(turn);

            telemetry.addData("Action", "Turning to %.1f°", targetDeg);
            telemetry.addData("Heading / Error", "%.1f° / %.1f°", getHeading(), error);
            telemetry.update();
        }
        stopDrive();
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        front_left.setMode(mode);
        front_right.setMode(mode);
        back_left.setMode(mode);
        back_right.setMode(mode);
    }

    private void setDrivePower(double p) {
        front_left.setPower(p);
        front_right.setPower(p);
        back_left.setPower(p);
        back_right.setPower(p);
    }

    private void stopDrive() {
        setDrivePower(0);
    }

    /** Rough timeout: expected time at this power, doubled, plus 1 s. */
    private double timeoutFor(double inches, double power) {
        double inchesPerSecAtFull = 50;  // rough for 312 RPM + 96 mm wheels
        return Math.abs(inches) / (inchesPerSecAtFull * Math.max(Math.abs(power), 0.1)) * 2 + 1;
    }

    // ══════════════════════════════════════════════════════════════════════════
    // MECHANISM HELPERS
    // ══════════════════════════════════════════════════════════════════════════

    /** Positive = intake in, negative = spit out, 0 = stop. */
    private void setIntake(double velocity) {
        intake.setVelocity(velocity);
    }

    /** Spins the shooter and waits until it is up to speed (or times out). */
    private void spinUpShooter(double velocity, double timeoutS) {
        shooter.setVelocity(velocity);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutS
                && Math.abs(shooter.getVelocity() - velocity) > SHOOTER_TOLERANCE) {
            telemetry.addData("Action", "Spinning up shooter");
            telemetry.addData("Shooter vel", "%.0f / %.0f", shooter.getVelocity(), velocity);
            telemetry.update();
        }
    }

    /** One push of the feeder servo. */
    private void fireOnce() {
        shooter_servo.setPosition(SERVO_FIRE);
        sleep(300);
        shooter_servo.setPosition(SERVO_REST);
        sleep(300);
    }

    /** Spins up, fires `shots` times (re-checking speed between shots), then stops. */
    private void shoot(int shots) {
        spinUpShooter(SHOOTER_VELOCITY, 2.0);
        for (int i = 0; i < shots && opModeIsActive(); i++) {
            fireOnce();
            spinUpShooter(SHOOTER_VELOCITY, 1.0);
        }
        shooter.setVelocity(0);
    }
}