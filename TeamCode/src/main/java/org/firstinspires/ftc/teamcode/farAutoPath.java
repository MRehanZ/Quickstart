package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class farAutoPath extends OpMode {

    final double FEED_TIME = 0.2;
    final double TIME_BETWEEN_SHOTS = 1.5;

    double TARGET_RPM = 1725;

    int shotsToFire = 3;

    private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime interShotTimer = new ElapsedTime();


    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;

    private enum LaunchState {
        IDLE,
        SPINUP,
        LAUNCH,
        BETWEEN_SHOTS
    }

    private LaunchState launchState;

    public enum PathState {
        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_INTAKEPOS,
        SHOOT_SET_ONE,
        DRIVE_HUMANPLAYER_TWO,
        SHOOT_SET_TWO,
        LEAVE_BASE
    }

    PathState pathState;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));

    private Pose shootPose = new Pose(56, 17, Math.toRadians(110));

    private final Pose intakePose = new Pose(120, 13.5, Math.toRadians(180));

    private final Pose endPose = new Pose(56, 17, Math.toRadians(110));

    // Leave position
    private final Pose leavePose = new Pose(36.5, 12, Math.toRadians(90));


    private PathChain driveStartPosShootPos, driveShootPosIntakePos, driveIntakePosEndPos, leavePath;


    public void buildPaths() {

        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();


        driveShootPosIntakePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intakePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakePose.getHeading())
                .build();


        driveIntakePosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(intakePose, endPose))
                .setLinearHeadingInterpolation(intakePose.getHeading(), endPose.getHeading())
                .build();


        leavePath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leavePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), leavePose.getHeading())
                .build();
    }


    public void statePathUpdate() {

        if (opModeTimer.getElapsedTimeSeconds() >= 27) {

            follower.followPath(driveStartPosShootPos, true);
            setPathState(PathState.LEAVE_BASE);
        }

        switch (pathState) {

            case DRIVE_STARTPOS_SHOOTPOS:

                follower.followPath(driveStartPosShootPos, true);

                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:

                if (!follower.isBusy()) {

                    if (launchState == LaunchState.IDLE) {

                        launchState = LaunchState.SPINUP;
                        shotsToFire = 3;
                    }

                    if (processLaunchState()) {

                        launcher.setPower(0);
                        leftFeeder.setPower(0);
                        rightFeeder.setPower(0);

                        telemetry.addLine("Finished Shooting!");

                        // Go to intake
                        follower.followPath(driveShootPosIntakePos, true);

                        setPathState(PathState.DRIVE_SHOOTPOS_INTAKEPOS);
                    }
                }

                break;


            case DRIVE_SHOOTPOS_INTAKEPOS:

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 3) {

                    telemetry.addLine("Reached Intake");

                    // Return to shooting position
                    follower.followPath(driveIntakePosEndPos, true);

                    setPathState(PathState.SHOOT_SET_ONE);
                }

                break;


            case SHOOT_SET_ONE:

                if (!follower.isBusy()) {

                    if (launchState == LaunchState.IDLE) {

                        launchState = LaunchState.SPINUP;
                        shotsToFire = 3;
                    }

                    if (processLaunchState()) {

                        launcher.setPower(0);
                        leftFeeder.setPower(0);
                        rightFeeder.setPower(0);

                        telemetry.addLine(
                                "Finished Set One");

                        follower.followPath(driveShootPosIntakePos, true);

                        setPathState(PathState.DRIVE_HUMANPLAYER_TWO);
                    }
                }

                break;


            case DRIVE_HUMANPLAYER_TWO:

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 3) {

                    telemetry.addLine("Reached Intake Again");

                    // Return to shooting position
                    follower.followPath(driveIntakePosEndPos, true);

                    setPathState(PathState.SHOOT_SET_TWO);
                }

                break;


            case SHOOT_SET_TWO:

                if (!follower.isBusy()) {

                    if (launchState == LaunchState.IDLE) {

                        launchState = LaunchState.SPINUP;
                        shotsToFire = 3;
                    }

                    if (processLaunchState()) {

                        launcher.setPower(0);
                        leftFeeder.setPower(0);
                        rightFeeder.setPower(0);

                        telemetry.addLine("Finished Set Two");

                        // Start leave path
                        follower.followPath(leavePath, true);

                        setPathState(PathState.LEAVE_BASE);
                    }
                }

                break;


            case LEAVE_BASE:

                if (!follower.isBusy()) {

                    launcher.setPower(0);
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);

                    telemetry.addLine("AUTO COMPLETE");
                }

                break;


            default:

                telemetry.addLine("No State Commanded");

                break;
        }
    }


    public void setPathState(PathState newState) {

        pathState = newState;
        pathTimer.resetTimer();
    }


    @Override
    public void init() {

        launchState = LaunchState.IDLE;

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setZeroPowerBehavior(BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
        launcher.setPower(0);

        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(155, 0, 0, 15.9)
        );

        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;


        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        follower.setPose(startPose);


        telemetry.addLine("Init completed");
    }


    @Override
    public void init_loop() {

        follower.update();
        Drawing.drawDebug(follower);

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
    }


    @Override
    public void start() {

        opModeTimer.resetTimer();

        setPathState(pathState);

        resetRuntime();
    }


    @Override
    public void loop() {

        follower.update();
        Drawing.drawDebug(follower);

        statePathUpdate();

        if (opModeTimer.getElapsedTimeSeconds() >= 10) {
            shootPose.setHeading(Math.toRadians(140));
        }


        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Launcher Velocity", launcher.getVelocity());
        telemetry.addData("Shots Left", shotsToFire);
        telemetry.addData("Launch State", launchState);
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("OpMode Time", opModeTimer.getElapsedTimeSeconds());
    }


    private boolean processLaunchState() {

        switch (launchState) {

            case IDLE:

                return false;


            case SPINUP:

                double currentRPM = launcher.getVelocity();

                if (currentRPM < TARGET_RPM) {

                    launcher.setVelocity(TARGET_RPM);

                }

                if (Math.abs(currentRPM - TARGET_RPM) / TARGET_RPM < 0.02) {

                    leftFeeder.setPower(1);
                    rightFeeder.setPower(1);

                    feederTimer.reset();

                    launchState = LaunchState.LAUNCH;
                }

                return false;


            case LAUNCH:

                if (feederTimer.seconds() >= FEED_TIME) {

                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);

                    interShotTimer.reset();

                    launchState = LaunchState.BETWEEN_SHOTS;
                }

                return false;


            case BETWEEN_SHOTS:

                if (interShotTimer.seconds() >= TIME_BETWEEN_SHOTS) {

                    shotsToFire--;


                    if (shotsToFire > 0) {

                        launchState = LaunchState.SPINUP;

                    } else {

                        launchState = LaunchState.IDLE;
                    }


                    return shotsToFire <= 0;
                }

                return false;
        }


        return false;
    }
}