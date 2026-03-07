package org.firstinspires.ftc.teamcode.opmode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.RaceCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.commands.ThreadedCommandScheduler;
import com.smartcluster.oracleftc.commands.WaitCommand;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
import com.smartcluster.oracleftc.utils.Performance;

import org.firstinspires.ftc.teamcode.subsystem.Robot;


import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicReference;

@SuppressWarnings("Convert2MethodRef")
@Config
@Autonomous
public class RedFar extends LinearOpMode {

    private final CommandScheduler scheduler = new CommandScheduler();
    private final MovingAverageFilter loopTimeFilter = new MovingAverageFilter(50);

    private static Action commandToAction(Command c) {
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    c.init();
                    initialized = true;
                }
                c.update();
                if (c.finished()) {
                    c.end(false);
                    return false;
                }
                return true;
            }
        };
    }

    private static Command actionToCommand(Action a) {
        return new Command() {
            private boolean finished = false;
            @Override
            public void init() { finished = true; }
            @Override
            public void update() {
                TelemetryPacket p = new TelemetryPacket();
                finished = !a.run(p);
            }
            @Override
            public boolean finished() { return finished; }
            @Override
            public void end(boolean interrupted) { super.end(interrupted); }
            @Override
            public Set<Subsystem> requires() { return super.requires(); }
        };
    }
    private final Pose2d cornerCoordinate = new Pose2d(61,61, Math.toRadians(-45));
    private final Pose2d startPose = new Pose2d(13,-59, Math.toRadians(90));
    private final Pose2d shootPose = new Pose2d(15,-55,Math.toRadians(-120));
    private final Pose2d stack1 = new Pose2d(30,-33.5,Math.toRadians(180));
    private final Pose2d stack2 = new Pose2d(30,-9.5,Math.toRadians(180));
    private final Pose2d gate = new Pose2d(57,0,Math.toRadians(90));
    private final Pose2d rotation = new Pose2d(28.6,-55,Math.toRadians(180));
    private final Pose2d down = new Pose2d(56.7,-55,Math.toRadians(180));//67
    private final Pose2d stack3 = new Pose2d(28,13.5,Math.toRadians(180));
    private final Pose2d endPose = new Pose2d(35, -56, Math.toRadians(90));


    public VelConstraint gateVel = (pose2dDual, posePath, v) -> 20;
    public VelConstraint slow = (pose2dDual, posePath, v) -> 15;
    public VelConstraint normal = (pose2dDual, posePath, v) -> 60;
//    public static PIDController pidController = new PIDController(0.0055, 0.0000, 0.00014);


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(this,true);

        scheduler.schedule(robot.update());
        Command.run(new SequentialCommand(
                robot.reset()
        ));

        SequentialAction autoAction = new SequentialAction
    (
                        // 3 bile

            robot.drive.actionBuilder(startPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(shootPose,Math.toRadians(70))
                .build(),
            commandToAction(
                new SequentialCommand(
                    robot.turret.blockShooter(),
                    robot.drive.toCorner(shootPose,cornerCoordinate),
                    robot.turret.setVelocityAndAngleByDist(shootPose, cornerCoordinate),
                    robot.turret.WaitForRPM(1000),
                    robot.turret.releaseShooter()
                )
            ),

                        // 6 bile
            robot.drive.actionBuilder(shootPose)
                .setTangent(Math.toRadians(60))
                .splineToLinearHeading(stack1, Math.toRadians(0))
                .build(),

        new RaceAction(
            robot.drive.actionBuilder(shootPose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(55,-33.5,Math.toRadians(180)), 0)
                .build(),
            commandToAction(
                new SequentialCommand(
                    robot.turret.blockShooter(),
                    robot.intake.intake(),
                    new WaitCommand(2000)
                )
            )
        ),

        robot.drive.actionBuilder(new Pose2d(55,-33.5,Math.toRadians(180)))
            .setTangent(Math.toRadians(200))
            .splineToLinearHeading(shootPose, Math.toRadians(220))
            .build(),

        commandToAction(
            new SequentialCommand(
                robot.drive.toCorner(shootPose,cornerCoordinate),
                robot.turret.setVelocityAndAngleByDist(shootPose,cornerCoordinate),
                robot.turret.WaitForRPM(1000),
                robot.turret.releaseShooter()
            )
        ),

                        // 9 bile
        robot.drive.actionBuilder(shootPose)
            .setTangent(Math.toRadians(80))
            .splineToLinearHeading(stack2, Math.toRadians(0))
            .build(),

        new RaceAction(
            robot.drive.actionBuilder(stack2)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(55,-9.5,Math.toRadians(180)), 0)
                .build(),
            commandToAction(
                new SequentialCommand(
                    robot.turret.blockShooter(),
                    robot.intake.intake(),
                    new WaitCommand(2000)
                )
            )
        ),
        new SequentialAction(
            robot.drive.actionBuilder(new Pose2d(55,-9.5,Math.toRadians(180)))
                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(gate, Math.toRadians(180),gateVel)
                .build(),
            commandToAction(new WaitCommand(1000)),
                robot.drive.actionBuilder(gate)
                    .setTangent(Math.toRadians(-160))
                    .splineToLinearHeading(shootPose,Math.toRadians(220))
                    .build()
        ),

        commandToAction(
            new SequentialCommand(
                robot.drive.toCorner(shootPose, cornerCoordinate),
                robot.turret.setVelocityAndAngleByDist(shootPose, cornerCoordinate),
                robot.turret.WaitForRPM(1000),
                robot.turret.releaseShooter()

            )
        ),

        // 12 bile
        robot.drive.actionBuilder(shootPose)
            .setTangent(Math.toRadians(0))
            .splineToLinearHeading(rotation, 0)
            .build(),

        new ParallelAction(
                robot.drive.actionBuilder(rotation)
                    .setTangent(Math.toRadians(0))
                    .splineToLinearHeading(down, 0)
                    .build(),
            commandToAction(
                new SequentialCommand(
                     robot.turret.blockShooter(),
                     robot.intake.intake(),
                     new WaitCommand(2000)
                )
            )

        ),

        robot.drive.actionBuilder(down)
            .setTangent(Math.toRadians(180))
            .splineToLinearHeading(shootPose,Math.toRadians(0))
            .build(),

            commandToAction(
                    new SequentialCommand(
                            robot.drive.toCorner(shootPose, cornerCoordinate),
                            robot.turret.setVelocityAndAngleByDist(shootPose, cornerCoordinate),
                            robot.turret.WaitForRPM(1000),
                            robot.turret.releaseShooter()

                    )
            ),

            //15 bile
            robot.drive.actionBuilder(shootPose)
                    .setTangent(Math.toRadians(0))
                    .splineToLinearHeading(rotation, 0)
                    .build(),

            new ParallelAction(
                    robot.drive.actionBuilder(rotation)
                            .setTangent(Math.toRadians(0))
                            .splineToLinearHeading(down, 0)
                            .build(),
                    commandToAction(
                            new SequentialCommand(
                                    robot.turret.blockShooter(),
                                    robot.intake.intake(),
                                    new WaitCommand(2000)
                            )
                    )

            ),

            robot.drive.actionBuilder(down)
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(shootPose,Math.toRadians(0))
                    .build(),

            commandToAction(
                    new SequentialCommand(
                            robot.drive.toCorner(shootPose, cornerCoordinate),
                            robot.turret.setVelocityAndAngleByDist(shootPose, cornerCoordinate),
                            robot.turret.WaitForRPM(1000),
                            robot.turret.releaseShooter()

                    )
            )
    );

        waitForStart();

        robot.drive.localizer.setPose(startPose);

        List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        Canvas c = new Canvas();

        autoAction.preview(c);

        boolean running = true;
        while (running && !isStopRequested()) {
            robot.read();

            TelemetryPacket p = new TelemetryPacket();
            p.fieldOverlay().getOperations().addAll(c.getOperations());
//            robot.drive.updatePoseEstimate();
            scheduler.update();
            running = autoAction.run(p);

            FtcDashboard.getInstance().sendTelemetryPacket(p);
            telemetry.addData("Current pose", robot.drive.localizer.getPose().value().position);


//            telemetry.addData("Dex Current", robot.storage.spindexer.getPosition().get(0));
//            telemetry.addData("Dex Target", robot.storage.spindexer.getTarget());

            telemetry.addData("Turret Velocity", robot.turret.getCurrentVelocity());

            telemetry.addData("hz", loopTimeFilter.update(1 / (Performance.loopTimeNano() / 1E9)));
            telemetry.update();
        }

        while (opModeIsActive()) {
            robot.read();

            scheduler.update();
            telemetry.update();
        }
    }
}
