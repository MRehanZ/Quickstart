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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class farAutoPathAI extends OpMode {

    final double FEED_TIME = 0.2;
    final double TIME_BETWEEN_SHOTS = 1.55;

    double TARGET_RPM = 1765;

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

    // Poses
    private final Pose startPose =
            new Pose(56, 8, Math.toRadians(90));

    private final Pose shootPose =
            new Pose(56, 17, Math.toRadians(110));

    private final Pose intakePose =
            new Pose(120, 10, Math.toRadians(0));

    private final Pose endPose =
            new Pose(56, 17, Math.toRadians(110));

    // Leave position
    private final Pose leavePose =
            new Pose(100, 17, Math.toRadians(110));


    // Paths
    private PathChain driveStartPosShootPos;
    private PathChain driveShootPosIntakePos;
    private PathChain driveIntakePosEndPos;
    private PathChain leavePath;


    public void buildPaths() {

        // Start position -> shooting position
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        shootPose.getHeading()
                )
                .build();


        // Shooting position -> intake position
        driveShootPosIntakePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intakePose))
                .setLinearHeadingInterpolation(
                        shootPose.getHeading(),
                        intakePose.getHeading()
                )
                .build();


        // Intake position -> shooting position
        driveIntakePosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(intakePose, endPose))
                .setLinearHeadingInterpolation(
                        intakePose.getHeading(),
                        endPose.getHeading()
                )
                .build();


        // Shooting position -> leave position
        leavePath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leavePose))
                .setLinearHeadingInterpolation(
                        shootPose.getHeading(),
                        leavePose.getHeading()
                )
                .build();
    }


    public void statePathUpdate() {

        switch (pathState) {

            case DRIVE_STARTPOS_SHOOTPOS:

                follower.followPath(
                        driveStartPosShootPos,
                        true
                );

                setPathState(
                        PathState.SHOOT_PRELOAD
                );

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

                        telemetry.addLine(
                                "Finished Preload Shooting"
                        );

                        // Go to intake
                        follower.followPath(
                                driveShootPosIntakePos,
                                true
                        );

                        setPathState(
                                PathState.DRIVE_SHOOTPOS_INTAKEPOS
                        );
                    }
                }

                break;


            case DRIVE_SHOOTPOS_INTAKEPOS:

                if (!follower.isBusy()) {

                    telemetry.addLine(
                            "Reached Intake"
                    );

                    // Return to shooting position
                    follower.followPath(
                            driveIntakePosEndPos,
                            true
                    );

                    setPathState(
                            PathState.SHOOT_SET_ONE
                    );
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
                                "Finished Set One"
                        );

                        // Go to intake again
                        follower.followPath(
                                driveShootPosIntakePos,
                                true
                        );

                        setPathState(
                                PathState.DRIVE_HUMANPLAYER_TWO
                        );
                    }
                }

                break;


            case DRIVE_HUMANPLAYER_TWO:

                if (!follower.isBusy()) {

                    telemetry.addLine(
                            "Reached Intake Again"
                    );

                    // Return to shooting position
                    follower.followPath(
                            driveIntakePosEndPos,
                            true
                    );

                    setPathState(
                            PathState.SHOOT_SET_TWO
                    );
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

                        telemetry.addLine(
                                "Finished Set Two"
                        );

                        // Start leave path
                        follower.followPath(
                                leavePath,
                                true
                        );

                        setPathState(
                                PathState.LEAVE_BASE
                        );
                    }
                }

                break;


            case LEAVE_BASE:

                if (!follower.isBusy()) {

                    launcher.setPower(0);
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);

                    telemetry.addLine(
                            "AUTO COMPLETE"
                    );
                }

                break;


            default:

                telemetry.addLine(
                        "No State Commanded"
                );

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

        // Get launcher
        launcher = hardwareMap.get(
                DcMotorEx.class,
                "launcher"
        );

        // Get feeders
        leftFeeder = hardwareMap.get(
                CRServo.class,
                "left_feeder"
        );

        rightFeeder = hardwareMap.get(
                CRServo.class,
                "right_feeder"
        );


        // Reverse left feeder
        leftFeeder.setDirection(
                DcMotorSimple.Direction.REVERSE
        );


        // Launcher brakes
        launcher.setZeroPowerBehavior(
                BRAKE
        );


        // Launcher uses encoder
        launcher.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );


        // Stop everything
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
        launcher.setPower(0);


        // Initial state
        pathState =
                PathState.DRIVE_STARTPOS_SHOOTPOS;


        // Timers
        pathTimer = new Timer();
        opModeTimer = new Timer();


        // Create follower
        follower =
                Constants.createFollower(hardwareMap);


        // Build paths
        buildPaths();


        // Set starting pose
        follower.setPose(startPose);


        telemetry.addLine(
                "Init completed"
        );

        telemetry.update();
    }


    @Override
    public void init_loop() {

        follower.update();

        telemetry.addData(
                "X",
                follower.getPose().getX()
        );

        telemetry.addData(
                "Y",
                follower.getPose().getY()
        );

        telemetry.addData(
                "Heading",
                Math.toDegrees(
                        follower.getPose().getHeading()
                )
        );

        telemetry.update();
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

        statePathUpdate();


        telemetry.addData(
                "Path State",
                pathState.toString()
        );

        telemetry.addData(
                "X",
                follower.getPose().getX()
        );

        telemetry.addData(
                "Y",
                follower.getPose().getY()
        );

        telemetry.addData(
                "Heading",
                Math.toDegrees(
                        follower.getPose().getHeading()
                )
        );

        telemetry.addData(
                "Path Time",
                pathTimer.getElapsedTimeSeconds()
        );

        telemetry.addData(
                "OpMode Time",
                opModeTimer.getElapsedTimeSeconds()
        );

        telemetry.addData(
                "Launcher Velocity",
                launcher.getVelocity()
        );

        telemetry.addData(
                "Shots Left",
                shotsToFire
        );

        telemetry.addData(
                "Launch State",
                launchState
        );

        telemetry.update();
    }


    private boolean processLaunchState() {

        switch (launchState) {

            case IDLE:

                return false;


            case SPINUP:

                double currentRPM =
                        launcher.getVelocity();

                double increment = 75;


                // Gradually increase launcher velocity
                if (currentRPM < TARGET_RPM) {

                    launcher.setVelocity(
                            Math.min(
                                    currentRPM + increment,
                                    TARGET_RPM
                            )
                    );
                }


                // Check if within 2% of target
                if (
                        Math.abs(
                                currentRPM - TARGET_RPM
                        ) / TARGET_RPM < 0.02
                ) {

                    leftFeeder.setPower(1);
                    rightFeeder.setPower(1);

                    feederTimer.reset();

                    launchState =
                            LaunchState.LAUNCH;
                }

                return false;


            case LAUNCH:

                if (
                        feederTimer.seconds()
                                >= FEED_TIME
                ) {

                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);

                    interShotTimer.reset();

                    launchState =
                            LaunchState.BETWEEN_SHOTS;
                }

                return false;


            case BETWEEN_SHOTS:

                if (
                        interShotTimer.seconds()
                                >= TIME_BETWEEN_SHOTS
                ) {

                    shotsToFire--;


                    if (shotsToFire > 0) {

                        launchState =
                                LaunchState.SPINUP;

                    } else {

                        launchState =
                                LaunchState.IDLE;
                    }


                    return shotsToFire <= 0;
                }

                return false;
        }


        return false;
    }
}