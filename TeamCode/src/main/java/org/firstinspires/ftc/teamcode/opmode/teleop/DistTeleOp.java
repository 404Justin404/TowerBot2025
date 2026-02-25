package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.fsm.FSM;
import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
import com.smartcluster.oracleftc.utils.Performance;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

import org.firstinspires.ftc.teamcode.calibration.ShooterCalibration;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import com.bylazar.telemetry.PanelsTelemetry;
@Config
//@TeleOp(group = "TeleOp")
public class DistTeleOp extends LinearOpMode {
    protected Pose2d cornerCoordinate = new Pose2d(60,63, Math.toRadians(-45));
    protected boolean isRed = true;
    private final CommandScheduler scheduler = new CommandScheduler();

    public enum TeleOpState {
        INIT,
        IDLE,
        INTAKE,
        OUTTAKE,
        PRESHOOT,
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
                robot.drive.driveFieldCentric(driverGamepad, isRed, cornerCoordinate),
                robot.turret.VelocityUpdate()
        );

        // Initialize FSM
        FSM<TeleOpState> fsm = FSM.<TeleOpState>builder()
                .initial(TeleOpState.INIT)

                // INIT -> IDLE: Reset robot to starting state
                .transition(TeleOpState.INIT, TeleOpState.IDLE, this::opModeIsActive,
                        new SequentialCommand(
                                robot.reset(),
                                new InstantCommand(() ->{
                                    robot.turret.setTracking(robot.drive, cornerCoordinate);
//                                    robot.turret.setTargetVelocity(1000);
                                })
                        ))

                .transition(TeleOpState.IDLE, TeleOpState.INTAKE, driverGamepad.left_bumper.down(),
                        robot.intake.intake())

                .transition(TeleOpState.INTAKE, TeleOpState.IDLE, driverGamepad.left_bumper.up(),
                        robot.intake.slowIntake())

                .transition(TeleOpState.IDLE, TeleOpState.OUTTAKE, driverGamepad.circle.down(),
                        robot.intake.outake())

                .transition(TeleOpState.OUTTAKE, TeleOpState.IDLE, driverGamepad.circle.up(),
                        robot.intake.slowIntake())

                .transition(TeleOpState.IDLE, TeleOpState.PRESHOOT, driverGamepad.dpad_down.pressed(),
                        new SequentialCommand(
                                robot.intake.intake(),
                                new InstantCommand(()->robot.turret.isAboutToShot.set(true))
                        ))

                .transition(TeleOpState.PRESHOOT, TeleOpState.SHOOT, () -> driverGamepad.right_trigger.get() >= 0.5,
                        new SequentialCommand(
                                robot.turret.WaitForRPM(2000),
                                new InstantCommand(robot.turret::releaseShooter),
                                robot.intake.intake()
                        ))

                .transition(TeleOpState.SHOOT, TeleOpState.IDLE, () -> driverGamepad.right_trigger.get() < 0.5,
                        new SequentialCommand(
                                new InstantCommand(() ->
                                {
                                    robot.turret.isAboutToShot.set(false);
//                                    robot.turret.setTargetVelocity(1000);
                                    robot.turret.blockShooter();
                                }),
                                robot.intake.slowIntake()
                        ))

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