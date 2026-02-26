package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.commands.WaitCommand;
import com.smartcluster.oracleftc.fsm.FSM;
import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
import com.smartcluster.oracleftc.utils.Performance;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

@Config
//@TeleOp(group = "TeleOp")
public class BaseTeleOp extends LinearOpMode {
    protected Pose2d cornerCoordinate;
    protected Pose2d endPose;
    protected Pose2d closeShoot;
    protected Pose2d farShoot;
    protected boolean isRed = false;
    protected boolean resetEncoder = false;
    private final CommandScheduler scheduler = new CommandScheduler();

    public static double closehood=0;
    public static double closevelo=2600;
    public static double farvelo=3000;
    public static double farhood=0.43;
    public static double waitforrpm=2000;
    public enum TeleOpState {
        INIT,
        IDLE,
        INTAKE,
        OUTTAKE,
        FAR_SHOOTING,
        CLOSE_SHOOTING,
        SHOOT,
        LIFT,
        GOJO
    }

    private TeleOpState CurrentState = TeleOpState.INIT;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = new Robot(this, isRed);
        ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1);

        robot.drive.localizer.setPose(MecanumDrive.currentPose);

        // Schedule continuous updates
        scheduler.schedule(
                robot.update(),
                robot.drive.driveFieldCentric(driverGamepad, isRed, cornerCoordinate)
        );

        // Initialize FSM
        FSM<TeleOpState> fsm = FSM.<TeleOpState>builder()
                .initial(TeleOpState.INIT)

                // INIT -> IDLE: Reset robot to starting state
                .transition(TeleOpState.INIT, TeleOpState.IDLE, () -> true,
                        new SequentialCommand(
                                robot.reset(),
                                new InstantCommand(() -> {
                                    robot.turret.setTargetVelocity(1000);
                                    robot.turret.setHoodAngle(0.0);
                                    robot.turret.blockShooter();
                                    robot.intake.idleIntake();
                                })
                        ))
// IDLE -> INTAKE: Start intake when left bumper pressed
                .transition(TeleOpState.IDLE, TeleOpState.INTAKE, driverGamepad.left_bumper.down(),
                        robot.intake.intake())

                .transition(TeleOpState.INTAKE, TeleOpState.IDLE, driverGamepad.left_bumper.up(),
                        robot.intake.idleIntake())

// IDLE -> OUTTAKE: Manual outtake with right bumper
                .transition(TeleOpState.IDLE, TeleOpState.OUTTAKE, driverGamepad.circle.down(),
                        robot.intake.outake())

                .transition(TeleOpState.OUTTAKE, TeleOpState.IDLE, driverGamepad.circle.up(),
                        robot.intake.idleIntake())

//                RIGHT BUMPER ESTE BOOST

                // IDLE -> FAR_SHOOTING: Prepare for far shooting
                .transition(TeleOpState.IDLE, TeleOpState.FAR_SHOOTING, driverGamepad.dpad_up.pressed(),
                        new SequentialCommand(
                                robot.intake.intake(),
                                new InstantCommand(() -> {
                                    robot.turret.setTargetVelocity(farvelo);
                                    robot.turret.setHoodAngle(farhood);

                                })))

                // IDLE -> CLOSE_SHOOTING: Prepare for close shooting
                .transition(TeleOpState.IDLE, TeleOpState.CLOSE_SHOOTING, driverGamepad.dpad_down.pressed(),
                                new SequentialCommand(
                                        robot.intake.intake(),
                                    new InstantCommand(() -> {
                                        robot.turret.setTargetVelocity(closevelo);
                                        robot.turret.setHoodAngle(closehood);

                                })))

                .transition(TeleOpState.CLOSE_SHOOTING, TeleOpState.SHOOT, () -> driverGamepad.right_trigger.get() >= 0.5,
                        new SequentialCommand(
                                robot.turret.WaitForRPM(waitforrpm),
                                new InstantCommand(robot.turret::releaseShooter),
                                robot.intake.intake(),
                                new InstantCommand(() -> robot.turret.setHoodAngle(closehood+0.02)),
                                new WaitCommand(100),
                                new InstantCommand(() -> robot.turret.setHoodAngle(closehood+0.03))
                                )
                )

                .transition(TeleOpState.FAR_SHOOTING, TeleOpState.SHOOT, () -> driverGamepad.right_trigger.get() >= 0.5,
                        new SequentialCommand(
                                robot.turret.WaitForRPM(waitforrpm),
                                new InstantCommand(robot.turret::releaseShooter),
                                robot.intake.intake(),
                                new InstantCommand(() -> robot.turret.setHoodAngle(farhood+0.02)),
                                new WaitCommand(100),
                                new InstantCommand(() -> robot.turret.setHoodAngle(farhood+0.03))
                        )
                )

                .transition(TeleOpState.SHOOT, TeleOpState.IDLE, () -> driverGamepad.right_trigger.get() < 0.5,
                        new SequentialCommand(
                                new InstantCommand(() -> {
                                    robot.turret.blockShooter();
                                    robot.turret.setTargetVelocity(1000);
                                    robot.turret.setHoodAngle(0);
                                }),
                                robot.intake.idleIntake()
                        ))

                // Emergency stop
//                .transition(TeleOpState.FAR_SHOOTING, TeleOpState.IDLE, driverGamepad.square.pressed(),
//                        new InstantCommand(() -> robot.turret.setTargetVelocity(500)))
//                .transition(TeleOpState.CLOSE_SHOOTING, TeleOpState.IDLE, driverGamepad.square.pressed(),
//                        new InstantCommand(() -> robot.turret.setTargetVelocity(500)))

                .transition(TeleOpState.IDLE, TeleOpState.LIFT, driverGamepad.dpad_right.down(),
                        new SequentialCommand(
                                new InstantCommand(() -> {
                                    robot.turret.setHoodAngle(0.0);
                                    robot.turret.blockShooter();
                                }),
                                robot.intake.stop(),
                                robot.lift.liftUp()
                        ))


                .transition(TeleOpState.LIFT, TeleOpState.GOJO, driverGamepad.dpad_right.up(),
                        robot.lift.hold())


                .build(scheduler);

        waitForStart();

        MovingAverageFilter loopTimeFilter = new MovingAverageFilter(100);

        while (opModeIsActive()) {
            robot.read();

            CurrentState = fsm.getCurrentState();

            // Telemetry
            telemetry.addData("Current State", CurrentState);
            telemetry.addData("Turret Velocity", robot.turret.getCurrentVelocity());

            telemetry.addData("x", robot.drive.localizer.getPose().position.x.get(0));
            telemetry.addData("y", robot.drive.localizer.getPose().position.y.get(0));
            telemetry.addData("heading (deg)", Math.toDegrees(robot.drive.localizer.getPose().heading.log().get(0)));
            telemetry.addData("Loop Time (hz)", loopTimeFilter.update(1 / (Performance.loopTimeNano() / 1E9)));
            telemetry.update();

            fsm.update();
            driverGamepad.process();
        }
    }
}