package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
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

@Autonomous(name = "AutoPedroRedS12627", group = "Autonomous")
public class AutoPedroRedS12627 extends LinearOpMode {

    // ── HARDWARE ──────────────────────────────────────────────────────────────
    // The four drive motors are NOT set up here: Pedro's Follower (built by
    // Constants.create) owns them, including their directions and modes.
    // Setting them here too would fight the follower.
    private DcMotorEx shooter, intake;
    private Follower follower;

    // ── MECHANISM CONSTANTS (tune on the robot) ───────────────────────────────
    static final double INTAKE_VELOCITY   = 1000;  // ticks/sec

    // ── POSES (Pedro field coordinates, inches, heading in degrees) ───────────
    // Each path starts where the previous one ended, so the separate
    // "pointNStart" poses from the original weren't needed.
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start  = poseFactory.of(56,      10,       90);
    private final Pose point1 = poseFactory.of(55.9982, 15.2818, 180);
    private final Pose point2 = poseFactory.of(55.955,  12.00,  180);
    private final Pose point3 = poseFactory.of(5.2139,  11.00,   180);
    private final Pose point4 = poseFactory.of(56.0195, 12.00,  90);
    private final Pose point5 = poseFactory.of(56.0191, 12.00,  90);
    private final Pose point6 = poseFactory.of(7.5154, 98.368, 90);

    // ══════════════════════════════════════════════════════════════════════════
    // THE ROUTINE
    // ══════════════════════════════════════════════════════════════════════════
    // Where to shoot and intake is a guess. Move the mechanism commands
    // to the spots in your path where they belong.
    public Command autoRoutine() {
        return sequential(

                parallel(
                        follow(follower, path1())
                ),


                follow(follower, path2()),

                // Run the intake while driving down the wall, then stop it.
                instant(() -> setIntake(INTAKE_VELOCITY)),
                follow(follower, path3()),
                instant(() -> setIntake(0)),

                follow(follower, path4()),

                follow(follower, path5()),

                follow(follower, path6())

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

        telemetry.addData("Status", "Initialized — press START");
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

        // Make sure mechanisms are off if the routine was cut short.
        intake.setVelocity(0);
    }

    // ══════════════════════════════════════════════════════════════════════════
    // SETUP (mechanisms only, taken from your original file)
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
    public Path path5() { return Paths.line(point4, point5).linear(point4, point5); }
    public Path path6() { return Paths.line(point5, point6).linear(point5, point6); }
    // ══════════════════════════════════════════════════════════════════════════
    // MECHANISM COMMANDS
    // ══════════════════════════════════════════════════════════════════════════

    /** Positive = intake in, negative = spit out, 0 = stop. */
    private void setIntake(double velocity) {
        intake.setVelocity(velocity);
    }
}