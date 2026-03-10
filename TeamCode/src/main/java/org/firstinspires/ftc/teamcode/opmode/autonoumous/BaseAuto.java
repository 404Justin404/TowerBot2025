
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

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.util.calibration.new_fsm.State;
import org.firstinspires.ftc.teamcode.util.calibration.new_fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.calibration.new_fsm.Transition;

import java.util.List;

// this one drives to a good point to "scout" for balls before driving to them. it seems that cam has not enough fov, so we are going to drive closer.
// TODO: maybe add a "safety" path: if not over two balls were collected, make a new path to drive to corner?
@Configurable
@Autonomous(name="Testing Auto", group="Auto")
public class BaseAuto extends LinearOpMode {

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
    private final Pose startPose = new Pose(42, 9, Math.toRadians(-90));
    private final Pose goalPose = new Pose(140.5,140.5,Math.toRadians(-135));
    private PathChain intakeCorner,intakePile1,  intakeThird,intakePile2,intakePile3,
                      shootCorner, shootThird, lookPile1,  shootPile1, lookPile2,  shootPile2, lookPile3,  shootPile3;

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

        lookPile1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(42, 9), new Pose(42, 12)))
                .setLinearHeadingInterpolation(Math.toRadians(-90),Math.toRadians(-135))
                .build();

        intakeCorner = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(42.650, 9.000), new Pose(42.650, 20.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootCorner = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(42.650, 9.000), new Pose(46, 15)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        intakeThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46, 9),
                                new Pose(40.000, 36.000),
                                new Pose(38.000, 36.000),
                                new Pose(13.000, 36.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootThird = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(13.000, 36.000), new Pose(50, 16)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
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
        follower.setMaxPower(0.3);
        follower.update();
        buildPaths();

        SequenceAuto = new SequentialCommand(
                PedroToCommand(lookPile1, true),

                intake.intake(),
                new InstantCommand(()->flywheel.setTargetVelocity(3000)),
                flywheel.WaitForRPM(2000),
                new InstantCommand(flywheel::releaseShooter),
                new WaitCommand(1000),
                new InstantCommand(flywheel::blockShooter),
                intake.slowIntake(),

                new WaitCommand(1000),
                intake.intake()
//                PedroToCommand(intakeCorner, false)
        );

//        stateMachine = new StateMachine(
//                // preload
//                new State()
//                        .maxTime(3000) // in case it takes too long
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> {
//
//                            follower.setMaxPower(1);
//                        })
//                        .transition(new Transition(() -> isFinished())),
//                // corner
//                new State()
//                        .onEnter(() -> {
//                            robot.intakeCommand.start();
//                            follower.followPath(intakeCorner, false);
//                        })
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> follower.followPath(shootCorner, true))
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> robot.shootCommand.start())
//                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
//                // third
//                new State()
//                        .onEnter(() -> {
//                            robot.intakeCommand.start();
//                            follower.followPath(intakeThird, false);
//                        })
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> follower.followPath(shootThird, true))
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> robot.shootCommand.start())
//                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
//                // pile 1
//                new State()
//                        .onEnter(() -> {
//                            robot.intakeCommand.start();
//                            follower.followPath(lookPile1, true);
//                        })
//                        // iteration 1: updating ONCE! not multiple times, so we go to corner originally
//                        .transition(new Transition(() -> follower.atParametricEnd())),
//                new State()
//                        .onEnter(() -> {
//                            double currentY = follower.getPose().getY();
//                            double currentX = follower.getPose().getX();
//                            intakePile1 = follower.pathBuilder()
//                                    .addPath(
//                                            new BezierCurve(
//                                                    new Pose(currentX, currentY),
//                                                    new Pose(9, MathUtil.clamp(currentY + optimalX, 9, 40))
//                                            )
//                                    )
//                                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                                    .setVelocityConstraint(20)
//                                    .setTValueConstraint(0.9)
//                                    .setHeadingConstraint(Math.toRadians(5))
//
//                                    .build();
//                            shootPile1 = follower.pathBuilder()
//                                    .addPath(
//                                            new BezierCurve(
//                                                    new Pose(9, MathUtil.clamp(currentY + optimalX, 9, 40)),
//                                                    new Pose(50, 16)
//                                            )
//                                    )
//                                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                                    .build();
//                            follower.breakFollowing();
//                            follower.followPath(intakePile1, false);
//                        })
//                        // iteration 3: just go till the end, the intake is literally like 10 inches away so it shouldnt move much
//                        .maxTime(2000)
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> follower.followPath(shootPile1, true))
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> robot.shootCommand.start())
//                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
//                // pile 2
//                new State()
//                        .onEnter(() -> {
//                            robot.intakeCommand.start();
//                            follower.followPath(lookPile2, true);
//                        })
//                        // iteration 1: updating ONCE! not multiple times, so we go to corner originally
//                        .transition(new Transition(() -> follower.atParametricEnd())),
//                new State()
//                        .onEnter(() -> {
//                            double optimalX = robot.vision.getLargestClusterX();
//                            double currentY = follower.getPose().getY();
//                            double currentX = follower.getPose().getX();
//                            intakePile2 = follower.pathBuilder()
//                                    .addPath(
//                                            new BezierCurve(
//                                                    new Pose(currentX, currentY),
//                                                    new Pose(9, MathUtil.clamp(currentY + optimalX, 9, 40))
//                                            )
//                                    )
//                                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                                    .setVelocityConstraint(20)
//                                    .setTValueConstraint(0.9)
//                                    .setHeadingConstraint(Math.toRadians(5))
//
//                                    .build();
//                            shootPile2 = follower.pathBuilder()
//                                    .addPath(
//                                            new BezierCurve(
//                                                    new Pose(9, MathUtil.clamp(currentY + optimalX, 9, 40)),
//                                                    new Pose(50, 16)
//                                            )
//                                    )
//                                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                                    .build();
//                            follower.breakFollowing();
//                            follower.followPath(intakePile1, false);
//                        })
//                        // iteration 3: just go till the end, the intake is literally like 10 inches away so it shouldnt move much
//                        .maxTime(2000)
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> follower.followPath(shootPile2, true))
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> robot.shootCommand.start())
//                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
//                // pile 3
//                new State()
//                        .onEnter(() -> {
//                            robot.intakeCommand.start();
//                            follower.followPath(lookPile3, true);
//                        })
//                        // iteration 1: updating ONCE! not multiple times, so we go to corner originally
//                        .transition(new Transition(() -> follower.atParametricEnd())),
//                new State()
//                        .onEnter(() -> {
//                            double optimalX = robot.vision.getLargestClusterX();
//                            double currentY = follower.getPose().getY();
//                            double currentX = follower.getPose().getX();
//                            intakePile3 = follower.pathBuilder()
//                                    .addPath(
//                                            new BezierCurve(
//                                                    new Pose(currentX, currentY),
//                                                    new Pose(9, MathUtil.clamp(currentY + optimalX, 9, 40))
//                                            )
//                                    )
//                                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                                    .setVelocityConstraint(20)
//                                    .setTValueConstraint(0.9)
//                                    .setHeadingConstraint(Math.toRadians(5))
//
//                                    .build();
//                            shootPile3 = follower.pathBuilder()
//                                    .addPath(
//                                            new BezierCurve(
//                                                    new Pose(9, MathUtil.clamp(currentY + optimalX, 9, 40)),
//                                                    new Pose(50, 16)
//                                            )
//                                    )
//                                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                                    .build();
//                            follower.breakFollowing();
//                            follower.followPath(intakePile3, false);
//                        })
//                        // iteration 3: just go till the end, the intake is literally like 10 inches away so it shouldnt move much
//                        .maxTime(2000)
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> follower.followPath(shootPile3, true))
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> robot.shootCommand.start())
//                        .transition(new Transition(() -> robot.shootCommand.isFinished()))
//
//        );

        waitForStart();
        Command.run(new ParallelCommand(flywheel.reset(),intake.stop()));
        follower.activateAllPIDFs();
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

            log("Actual Localizer Pose", follower.getPose().toString());            log("Pose x", follower.getPose().getX());
            log("Pose y", follower.getPose().getY());
            log("Current heading", Math.toDegrees(follower.getPose().getHeading()));
            log("Turret Velocity", flywheel.getCurrentVelocity());
            log("hz", loopTimeFilter.update(1 / (Performance.loopTimeNano() / 1E9)));

            telemetry.update();
        }
    }
 }

