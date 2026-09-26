package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Drawing;

@Autonomous
public class SampleAutoPathing extends OpMode {

    final double FEED_TIME = 0.2;               // How long the feeder runs to push a ball
    final double TIME_BETWEEN_SHOTS = 2.0;      // Delay between shots

    int shotsToFire = 3;                        // How many shots the robot should fire

    private ElapsedTime feederTimer = new ElapsedTime();       // Timer for feeder duration
    private ElapsedTime interShotTimer = new ElapsedTime();    // Timer between shots


    private Follower follower;
    private Timer pathTimer, opModeTimer;

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

    public void statepathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(driveShootPosIntakePos, true);
                    telemetry.addLine("Done Path 1");
                    setPathState(PathState.DRIVE_SHOOTPOS_INTAKEPOS);
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

        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add in any other init mechanisms

        buildPaths();
        follower.setPose(startPose);

    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {

        follower.update();
        statepathUpdate();

        Drawing.drawDebug(follower);

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("OpMode Time", opModeTimer.getElapsedTimeSeconds());


    }
}