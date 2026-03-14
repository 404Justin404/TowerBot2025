package org.firstinspires.ftc.teamcode.opmode.autonoumous;

import android.media.audiofx.Visualizer;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.commands.WaitCommand;
import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
import com.smartcluster.oracleftc.utils.Performance;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.pedroPathing.SmartLocalizerPedro;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

import java.util.List;

@Configurable
@Autonomous(name="Blue Human Auto", group="Auto")
public class BlueHumanClose extends LinearOpMode {

    private static Command PedroToCommand(PathChain path, boolean holdEnd) {
        return Command.builder()
                .init(() ->
                {
                    follower.followPath(path, holdEnd);
                })
                .finished(() -> !follower.isBusy())
                .build();
    }

//    private static Command waitForHeading(double toleranceErr)
//    {
//        return Command.builder()
//                .build();
//    }

    List<LynxModule> lynxModules;


    private static Follower follower;
    private final CommandScheduler scheduler = new CommandScheduler();
    private final MovingAverageFilter loopTimeFilter = new MovingAverageFilter(50);
    private TelemetryManager panelsTelemetry;
    private static  SequentialCommand SequenceAuto, SequenceShoot, SequenceGate;
    protected boolean isRed = false;
    private final Pose startPose = new Pose(25.045310853530054, 128.32876712328766, Math.toRadians(-37));
    private final Pose goalPose = new Pose(4.5,140.5,Math.toRadians(-45));
    private final double shootPower = 2200;
    private final double turretPitch = 0.05;

    public PathChain PreShoot;
    public PathChain Stack2;
    public PathChain Stack2Shoot;
    public PathChain GateOpen;
    public PathChain GateShoot;
    public PathChain Stack3;
    public PathChain GateIn,GateIntermediary;
    public PathChain Stack3Shoot;

    public PathChain HumanIntake;
    public PathChain HumanShoot;
    public PathChain PureHumaIn;
    public PathChain PureHumanOut;
    public PathChain HumanINTAKE;
    public PathChain HumanOUT;

    public PathChain GateRelease;
    public PathChain GateINT;
    public PathChain GateOut;
    public PathChain Stack1;
    public PathChain Stack1Shoot;

    public PathChain Gate;
    public PathChain ReleaseGate;
    public PathChain ShootRelease;

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
                                new Pose(25.045, 128.329),
                                new Pose(58.428, 85.012)
                        )
                )
                .setVelocityConstraint(50)
                .setLinearHeadingInterpolation(Math.toRadians(-37), Math.toRadians(-45))
                .build();

        Stack3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(58.428, 85.012),
                                new Pose(43.329, 83.856),
                                new Pose(26.5, 84.467)
                        )
                )
//                .setBrakingStart(0.8)
                .setTangentHeadingInterpolation()
                .build();

        Stack3Shoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(26.5, 84.467),
                                new Pose(58.428, 85.012)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-50.5))
                .build();


        Stack2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(58.428, 85.012),
                                new Pose(48.854, 58.743),
                                new Pose(18.531, 58)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Stack2Shoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(18.531, 58),
                                new Pose(58.428, 85.012)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-50.5))
                .build();

        GateIntermediary = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(58.428, 85.012),
                                new Pose(52.852, 58.8),
                                new Pose(33, 59.5)
                        )
                )

//                .setLinearHeadingInterpolation(Math.toRadians(-51), Math.toRadians(150),0.7)
                .setLinearHeadingInterpolation(Math.toRadians(-50.5), Math.toRadians(155))
                .setVelocityConstraint(55)
                .build();

        GateIn = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(20.516, 69.083),
                                new Pose(23,55),
                                new Pose(15.5, 12)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-120))
                .build();

        GateOut = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(18.5, 12),
                                new Pose(58.428, 85.012)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-120), Math.toRadians(-50.5))
                .build();

//        GateINT = follower.pathBuilder()
//                .addPath(
//                        new BezierCurve(
//                                new Pose(58.880, 84.536),
//                                new Pose(32.259, 48.711),
//                                new Pose(14.886, 59.346)
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .build();

        GateRelease = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(58.428, 85.012),
                                new Pose(20.516, 69.083)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
                .build();
//        GateShoot = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Pose(16.615, 68.693),
//                                new Pose(58.428, 85.012)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-52))
//                .build();

        HumanIntake = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(15.516, 69.083),
                                new Pose(20.850, 64.337),
                                new Pose(9.232, 9.322)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-135))
                .build();
        HumanShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(9.232, 9.322),
                                new Pose(58.428, 85.012)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-115), Math.toRadians(-45))
                .build();
        Stack1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(58.643, 84.702),
                                new Pose(66.450, 31.558),
                                new Pose(14.203, 35.565)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Stack1Shoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(14.203, 35.565),
                                new Pose(58.803, 84.498)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-50.5))
                .build();
        HumanINTAKE = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(58.913, 84.937),
                                new Pose(48.174, 45.816),
                                new Pose(11.846, 33.393),
                                new Pose(9.212, 10.418)
                        )
                )
                .setConstantHeadingInterpolation(-125)
                .build();

        HumanOUT = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(9.212, 10.418),
                                new Pose(59.033, 84.916)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-125), Math.toRadians(-45))
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        flywheel = new Turret(this);
        intake = new Intake(this);
        drive = new MecanumDrive(hardwareMap,telemetry);


        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.9);
        follower.update();
        buildPaths();

        SmartLocalizerPedro localizerPedro = (SmartLocalizerPedro) follower.getPoseTracker().getLocalizer();

        // Repetitive Command
        SequenceShoot = new SequentialCommand(
                intake.intake(),
                flywheel.velAuto(shootPower,turretPitch),
                flywheel.WaitForRPM(1000),
                new InstantCommand(flywheel::releaseShooter),
                new WaitCommand(550),
                new InstantCommand(flywheel::blockShooter),
                intake.slowIntake()
        );

        SequenceGate = new SequentialCommand(
                new ParallelCommand(
                        intake.intake(),
                        PedroToCommand(HumanINTAKE,true)
                ),
                PedroToCommand(HumanOUT,true)
        );



        SequenceAuto = new SequentialCommand(
                PedroToCommand(PreShoot, true),
                SequenceShoot,

                //END OF PRESHOOT
                new ParallelCommand(
                        intake.intake(),
                        PedroToCommand(Stack3,false)
                ),

                PedroToCommand(Stack3Shoot,true),
                SequenceShoot,
                //END OF STACK 3

                new ParallelCommand(
                        intake.intake(),
                        PedroToCommand(Stack2,false)
                ),

                PedroToCommand(Stack2Shoot,true),
                SequenceShoot,
                //END OF STACK 2

                PedroToCommand(GateRelease,true),
                new WaitCommand(1000),
                new ParallelCommand(
                        intake.intake(),
                        PedroToCommand(GateIn,true)
                ),

                PedroToCommand(GateOut,true)  ,
                SequenceShoot,

                SequenceGate,
                SequenceShoot,
                SequenceGate,
                SequenceShoot,


                intake.stop(),
                new InstantCommand(flywheel::disable)
                //END OF STACK 1
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


            localizerPedro.getTelemetry(telemetry);
//            log("Actual Localizer Pose", follower.getPose().toString());
            log("Pose x", follower.getPose().getX());
            log("Pose y", follower.getPose().getY());
            log("Current heading", Math.toDegrees(follower.getPose().getHeading()));
            log("Turret Velocity", flywheel.getCurrentVelocity());
            log("hz", loopTimeFilter.update(1 / (Performance.loopTimeNano() / 1E9)));

            telemetry.update();
        }
    }
}

