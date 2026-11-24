package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPedroBlueS212627", group = "Autonomous")
public class AutoPedroBlueS212627 extends LinearOpMode {

    // ── HARDWARE ──────────────────────────────────────────────────────────────
    // Drive motors are owned by Pedro's Follower (Constants.create).
    private DcMotorEx shooter, intake;
    private Follower follower;

    // ── MECHANISM CONSTANTS (tune on the robot) ───────────────────────────────
    static final double INTAKE_VELOCITY = 1000;  // ticks/sec

    // ── RED → BLUE CONVERSION ─────────────────────────────────────────────────
    // Blue is the red path rotated 180° around the field, so the robot makes
    // exactly the same moves relative to itself (same turns, same distances).
    //
    // The pivot point is chosen so the red start (57, 130, 270) lands exactly
    // on the blue start (82, 10, 90):
    //     blue x       = 2 * PIVOT_X - red x   = 139 - red x
    //     blue y       = 2 * PIVOT_Y - red y   = 140 - red y
    //     blue heading = red heading + 180
    static final double PIVOT_X = 69.5;  // midpoint of 57 and 82
    static final double PIVOT_Y = 70;    // midpoint of 130 and 10

    private final PoseFactory poseFactory = PoseFactory.degrees();

    /** Takes a RED pose and returns the matching BLUE pose. */
    private Pose toBlue(double redX, double redY, double redHeadingDeg) {
        double h = (redHeadingDeg + 180) % 360;
        return poseFactory.of(2 * PIVOT_X - redX, 2 * PIVOT_Y - redY, h);
    }

    // ── POSES: red numbers copied as-is, converted by toBlue() ────────────────
    // Resulting blue poses:
    //   start  ( 82.0,  10.0,  90)
    //   point1 ( 82.0,  31.5,  90)
    //   point2 ( 92.5,  14.5, 270)
    //   point3 ( 82.0,  14.5,  90)
    //   point4 (127.0,  25.0,  90)
    private final Pose start  = toBlue(57.00, 130.00, 270);
    private final Pose point1 = toBlue(57.00, 108.50, 270);
    private final Pose point2 = toBlue(46.50, 125.50, 90);
    private final Pose point3 = toBlue(57.00, 125.50, 270);
    private final Pose point4 = toBlue(12.00, 115.00, 270);

    // ══════════════════════════════════════════════════════════════════════════
    // THE ROUTINE (same order as the red auto)
    // ══════════════════════════════════════════════════════════════════════════
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),

                // Run the intake while driving, then stop it.
                instant(() -> setIntake(INTAKE_VELOCITY)),
                follow(follower, path2()),
                instant(() -> setIntake(0)),

                follow(follower, path3()),
                follow(follower, path4())
        );
    }

    // ══════════════════════════════════════════════════════════════════════════
    @Override
    public void runOpMode() {
        initMechanisms();

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        telemetry.addData("Status", "BLUE — Initialized, press START");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            shooter.setVelocity(1500);
            Scheduler.execute();

            telemetry.addData("x", "%.1f", follower.pose().x());
            telemetry.addData("y", "%.1f", follower.pose().y());
            telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(follower.pose().heading()));
            if (follower.currentPath() != null) {
                telemetry.addData("Path number", follower.pathIndex());
                telemetry.addData("Distance remaining", "%.1f", follower.distanceToEndpoint());
            }
            telemetry.addData("Intake vel", "%.0f", intake.getVelocity());
            telemetry.update();
        }

        intake.setVelocity(0);
    }

    // ══════════════════════════════════════════════════════════════════════════
    // SETUP
    // ══════════════════════════════════════════════════════════════════════════
    private void initMechanisms() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ══════════════════════════════════════════════════════════════════════════
    // PATHS
    // ══════════════════════════════════════════════════════════════════════════
    public Path path1() { return Paths.line(start,  point1).linear(start,  point1); }
    public Path path2() { return Paths.line(point1, point2).linear(point1, point2); }
    public Path path3() { return Paths.line(point2, point3).linear(point2, point3); }
    public Path path4() { return Paths.line(point3, point4).linear(point3, point4); }

    // ══════════════════════════════════════════════════════════════════════════
    // MECHANISM COMMANDS
    // ══════════════════════════════════════════════════════════════════════════

    /** Positive = intake in, negative = spit out, 0 = stop. */
    private void setIntake(double velocity) {
        intake.setVelocity(velocity);
    }
}
