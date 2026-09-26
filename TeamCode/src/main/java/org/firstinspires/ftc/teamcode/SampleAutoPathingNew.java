package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class SampleAutoPathingNew extends OpMode {


    final double FEED_TIME = 0.2;               // How long the feeder runs to push a ball
    final double TIME_BETWEEN_SHOTS = 2.0;      // Delay between shots

    double TARGET_RPM = 1350;

    int shotsToFire = 3;                        // How many shots the robot should fire

    private ElapsedTime feederTimer = new ElapsedTime();       // Timer for feeder duration
    private ElapsedTime interShotTimer = new ElapsedTime();    // Timer between shots


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
        // START POSITION_END POSITION
        // DRIVE -> MOVEMENT
        // SHOOT -> ATTEMPT TO SCORE THE ARTIFACT

        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_INTAKEPOS,
        DRIVE_INTAKEPOS_ENDPOS
    }

    PathState pathState;

    private final Pose startPose = new Pose(20, 121, Math.toRadians(137));
    private final Pose shootPose = new Pose(52.5, 89, Math.toRadians(137));
    private final Pose intakePose = new Pose(38.9, 82.7, Math.toRadians(180));
    private final Pose endPose = new Pose(60, 105.5, Math.toRadians(90));

    private PathChain driveStartPosShootPos, driveShootPosIntakePos, driveIntakePosEndPos;

    public void buildPaths() {
        // put in coordinates for starting pose -> ending pose
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
    }

    public void statePathUpdate() {
        switch (pathState) {

            case DRIVE_STARTPOS_SHOOTPOS:

                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);

                break;


            case SHOOT_PRELOAD:

                // Wait until robot reaches shooting position
                if (!follower.isBusy()) {

                    // Start shooting sequence
                    if (launchState == LaunchState.IDLE) {
                        launchState = LaunchState.SPINUP;
                        shotsToFire = 3;
                    }

                    // Process shooting
                    if (processLaunchState()) {

                        launcher.setPower(0);
                        leftFeeder.setPower(0);
                        rightFeeder.setPower(0);

                        telemetry.addLine("Finished Shooting");

                        // Now drive to intake position
                        follower.followPath(driveShootPosIntakePos, true);
                        setPathState(PathState.DRIVE_SHOOTPOS_INTAKEPOS);
                    }
                }

                break;


            case DRIVE_SHOOTPOS_INTAKEPOS:

                if (!follower.isBusy()) {

                    follower.followPath(driveIntakePosEndPos, false);

                    telemetry.addLine("Done Path 2");

                    setPathState(PathState.DRIVE_INTAKEPOS_ENDPOS);
                }

                break;


            case DRIVE_INTAKEPOS_ENDPOS:

                if (!follower.isBusy()) {
                    telemetry.addLine("Done All Paths");
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

        launchState = LaunchState.IDLE;          // Launcher starts idle

        // Get launcher and feeders
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        // Reverse lift side motors/servos
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motors to brake when power = 0
        launcher.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Launcher uses velocity control

        leftFeeder.setPower(0);  // Feeders stopped
        rightFeeder.setPower(0);
        launcher.setPower(0);    // Launcher stopped

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

        Drawing.drawDebug(follower);

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("OpMode Time", opModeTimer.getElapsedTimeSeconds());

        telemetry.addData("Launcher RPM", launcher.getVelocity());  // Show flywheel RPM
        telemetry.addData("Shots left", shotsToFire);               // Show remaining shots
        telemetry.addData("Launch State: ", launchState);


    }

    private boolean processLaunchState() {  // Handles launcher shot sequence
        switch (launchState) {
            case IDLE:
                return false;

            case SPINUP:
                double currentRPM = launcher.getVelocity(); // Get current launcher speed
                double increment = 50;                      // How much speed increases per loop

                // Gradually increase velocity
                if (currentRPM < TARGET_RPM) {
                    launcher.setVelocity(Math.min(currentRPM + increment, TARGET_RPM));
                }

                // If speed is within 2% target range
                if (Math.abs(currentRPM - TARGET_RPM) / TARGET_RPM < 0.02) {
                    leftFeeder.setPower(1);     // Feed ball
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
                    shotsToFire--;  // Decrease shot counter

                    if (shotsToFire > 0)
                        launchState = LaunchState.SPINUP; // Spin up again for next shot
                    else
                        launchState = LaunchState.IDLE;   // Done shooting

                    return shotsToFire <= 0;
                }
                return false;
        }
        return false;
    }
}
