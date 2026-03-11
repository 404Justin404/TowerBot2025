
package org.firstinspires.ftc.teamcode.opmode.autonoumous;



import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.commands.WaitCommand;
import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
import com.smartcluster.oracleftc.utils.Performance;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.util.calibration.new_fsm.State;
import org.firstinspires.ftc.teamcode.util.calibration.new_fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.calibration.new_fsm.Transition;

import java.util.List;
import java.util.Stack;

// this one drives to a good point to "scout" for balls before driving to them. it seems that cam has not enough fov, so we are going to drive closer.
// TODO: maybe add a "safety" path: if not over two balls were collected, make a new path to drive to corner?
@Configurable
@Autonomous(name="Tuning Auto", group="Auto")
public class AutoTuning extends LinearOpMode {

    private static Command PedroToCommand(PathChain path, boolean holdEnd) {
        return Command.builder()
                .init(() ->
                {
                    follower.followPath(path, holdEnd);
                })
                .finished(() -> !follower.isBusy())
                .build();
    }
    List<LynxModule> lynxModules;


    private static Follower follower;
    private final CommandScheduler scheduler = new CommandScheduler();
    private final MovingAverageFilter loopTimeFilter = new MovingAverageFilter(50);
    private TelemetryManager panelsTelemetry;
    private SequentialCommand SequenceAuto;
    protected boolean isRed = false;
    private final Pose startPose = new Pose(83.616, 16.577, Math.toRadians(-90));
    private final Pose goalPose = new Pose(140.5,140.5,Math.toRadians(-135));
    //    private PathChain intakeCorner,intakePile1,  intakeThird,intakePile2,intakePile3,
//                      shootCorner, shootThird, lookPile1,  shootPile1, lookPile2,  shootPile2, lookPile3,  shootPile3;
    public PathChain PreShoot;
    public PathChain Stack2;
    public PathChain Intake2;
    public PathChain Shoot2;
    public PathChain GateOpen;
    public PathChain GateShoot;
    public PathChain Stack1;
    public PathChain Stack1Shoot;
    public PathChain HumanIntake;
    public PathChain HumanShoot;

    private Turret flywheel;
    private MecanumDrive drive;
    private Intake intake;

    private void log(String caption, Object... text) {
        if (text.length == 1) {
            telemetry.addData(caption, text[0]);
            panelsTelemetry.debug(caption + ": " + text[0]);
        } else if (text.length >= 2) {
            StringBuilder message = new StringBuilder();
            for (int i = 0; i < text.length; i++) {
                message.append(text[i]);
                if (i < text.length - 1) message.append(" ");
            }
            telemetry.addData(caption, message.toString());
            panelsTelemetry.debug(caption + ": " + message);
        }
    }

    public void buildPaths() {

        PreShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(79.519, 9.214),
                                new Pose(83.616, 16.577)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-123))
                .build();

        Stack2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(83.616, 16.577),
                                new Pose(83.472, 59.051),
                                new Pose(102.086, 59.268)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-123), Math.toRadians(0))
                .build();

        Intake2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(102.086, 59.268),
                                new Pose(134.241, 59.268)
                        )
                )
                .setBrakingStart(0.81)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(134.241, 59.268),
                                new Pose(83.616, 16.577)
                        )
                )
                .setBrakingStart(0.81)
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-123))
                .build();

        GateOpen = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(83.616, 16.577),
                                new Pose(90.001, 60.643),
                                new Pose(133.075, 56.902)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-123), Math.toRadians(0))
                .build();

        GateShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(133.075, 56.902),
                                new Pose(83.571, 16.427)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-123))
                .build();

        Stack1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(83.571, 16.427),
                                new Pose(92.805, 38.366),
                                new Pose(131.352, 35.377)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Stack1Shoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(131.352, 35.377),
                                new Pose(83.498, 16.812)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        HumanIntake = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(83.498, 16.812),
                                new Pose(135.861, 22.355),
                                new Pose(135.322, 9.468)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        HumanShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(135.322, 9.468),
                                new Pose(83.041, 16.701)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-85), Math.toRadians(-123))
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        flywheel = new Turret(this);
        intake = new Intake(this);
        drive = new MecanumDrive(hardwareMap,telemetry);



        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        sleep(200);
        follower.update();
        buildPaths();

        SequenceAuto = new SequentialCommand(
                PedroToCommand(Stack2, true),
               new WaitCommand(1000),
               PedroToCommand(Shoot2,true),
                new WaitCommand(1000),
                PedroToCommand(Stack2, true),
               new WaitCommand(1000),
               PedroToCommand(Shoot2,true),
                new WaitCommand(1000),
                PedroToCommand(Stack2, true),
                new WaitCommand(1000),
                PedroToCommand(Shoot2,true),
                new WaitCommand(1000),
                PedroToCommand(Stack2, true),
                new WaitCommand(1000),
                PedroToCommand(Shoot2,true),
                new WaitCommand(1000),
                PedroToCommand(Stack2, true),
                new WaitCommand(1000),
                PedroToCommand(Shoot2,true),
                new WaitCommand(1000),
                PedroToCommand(Stack2, true),
                new WaitCommand(1000),
                PedroToCommand(Shoot2,true),
                new WaitCommand(1000),
                PedroToCommand(Stack2, true),
                new WaitCommand(1000),
                PedroToCommand(Shoot2,true),
                new WaitCommand(1000),
                PedroToCommand(Stack2, true),
                new WaitCommand(1000),
                PedroToCommand(Shoot2,true),
                new WaitCommand(1000),
                PedroToCommand(Stack2, true),
                new WaitCommand(1000),
                PedroToCommand(Shoot2,true)
//                PedroToCommand(intakeCorner, false)
        );


        waitForStart();
        Command.run(new ParallelCommand(flywheel.reset(),intake.stop()));
        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        follower.update();

        lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        scheduler.schedule(SequenceAuto, flywheel.update());

        while(opModeIsActive()){

            for (LynxModule lynxModule : lynxModules) {
                lynxModule.clearBulkCache();
                lynxModule.getBulkData();
            }

            scheduler.update();
            follower.update();
            panelsTelemetry.update();

            log("Target Pose", follower.getCurrentPath().endPose());
            log("Intake voltage",intake.getCurrentAmps());
            intake.BallNumber(intake.getCurrentAmps());
            log("Actual Localizer Pose", follower.getPose().toString());
            log("Pose x", follower.getPose().getX());
            log("Pose y", follower.getPose().getY());
            log("Current heading", Math.toDegrees(follower.getPose().getHeading()));
            log("Turret Velocity", flywheel.getCurrentVelocity());
            log("hz", loopTimeFilter.update(1 / (Performance.loopTimeNano() / 1E9)));

            telemetry.update();
        }
    }
}

