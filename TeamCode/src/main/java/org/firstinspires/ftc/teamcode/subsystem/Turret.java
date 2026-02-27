package org.firstinspires.ftc.teamcode.subsystem;
//iar avem variabile care vor modificate
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.hardware.subsystem.ServoActuator;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.hardware.subsystem.SubsystemFlavor;
import com.smartcluster.oracleftc.math.control.MotorFeedforward;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;
import com.smartcluster.oracleftc.math.filters.LowPassFilter;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

@Config
public class Turret extends Subsystem {

    // Hardware
    private final DcMotorImplEx shooter1, shooter2;
    private final ServoImplEx servoHood, servoLever;
    private final OracleLynxVoltageSensor voltageSensor;

    public MecanumDrive drive;
    private Pose2d goal;

    public AtomicBoolean enabledVel = new AtomicBoolean(true);
    public AtomicBoolean isAboutToShot = new AtomicBoolean(false);

    // Motion profiles
//    public static TrapezoidalMotionProfile hoodMotionProfile = new TrapezoidalMotionProfile(30, 30, 30);
//    public static TrapezoidalMotionProfile leverMotionProfile = new TrapezoidalMotionProfile(30, 30, 30);
//
//    // Servo actuators
//    public final ServoActuator hood, lever;

    public static MotorFeedforward flywheelFeedforward = new MotorFeedforward(0.201502, 0.000154, 0);
    public static PIDController flywheelPID = new PIDController(0.005, 0, 0.00035, 0.5);
    public static LowPassFilter velocityFilter = new LowPassFilter(0.5);
    private double targetVelocity = 0; // RPM
    public static double RPM_TOLERANCE = 100;


    // Velocity linear regression constants
// Formula: velocity = VELOCITY_SLOPE * distance_cm + VELOCITY_INTERCEPT
    private static final double VELOCITY_SLOPE = 4.397090;
    private static final double VELOCITY_INTERCEPT = 1444.533484;

    // Hood angle linear regression constants
// Formula: hood = HOOD_SLOPE * velocity + HOOD_INTERCEPT
    private static final double HOOD_SLOPE = 0.000179;
    private static final double HOOD_INTERCEPT = -0.319426;

    private boolean inZone;

    public static double HOOD_MIN_POSITION = 0.0;
    public static double HOOD_MAX_POSITION = 0.35;
    public static double LEVER_BLOCK_POSITION = 0.15;  // Position that blocks the flywheel
    public static double LEVER_RELEASE_POSITION = 0.0; // Position that allows shooting

    public Turret(OpMode opMode) {
        super(opMode);

        voltageSensor = hardwareMap.getAll(OracleLynxVoltageSensor.class).iterator().next();

        // Initialize flywheel motors
        shooter1 = hardwareMap.get(DcMotorImplEx.class, "shooter1");
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter2 = hardwareMap.get(DcMotorImplEx.class, "shooter2");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize servos
        servoHood = hardwareMap.get(ServoImplEx.class, "hood");
        servoHood.setDirection(Servo.Direction.REVERSE);
        servoLever = hardwareMap.get(ServoImplEx.class, "lever");
    }

    public void setTracking(MecanumDrive drive, Pose2d goal)
    {
        this.drive = drive;
        this.goal = goal;
    }

    /**
     * Get current flywheel velocity in RPM
     */
    public double getCurrentVelocity() {
        return velocityFilter.update((shooter2.getVelocity() / 28) * 60);
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = Math.max(0, Math.min(6000, velocity));
    }

    public Supplier<Boolean> isInsideTheZone(Pose2d pose)
    {
        double BigTriangle = Math.abs(pose.position.x)-4;
        double TinyTriangle = -Math.abs(pose.position.x)-7;
        return () -> pose.position.y >= BigTriangle || pose.position.y <= TinyTriangle;
    }

    public void blockShooter() {
        servoLever.setPosition(LEVER_BLOCK_POSITION);
    }

    public void releaseShooter() {
        servoLever.setPosition(LEVER_RELEASE_POSITION);
    }

    public void setLeverAngle(double angle){
        servoLever.setPosition(angle);
    }
    public void setHoodAngle(double angle)
    {
        double target = Math.max(HOOD_MIN_POSITION, Math.min(HOOD_MAX_POSITION, angle));
        servoHood.setPosition(target);
    }

    public double getHoodAngle()
    {
        return servoHood.getPosition();
    }

    public double getDistanceToTarget(Pose2d currPos, com.acmerobotics.roadrunner.Pose2d corner){
        double currentX = currPos.position.x;
        double currentY = currPos.position.y;

        double dx = corner.position.x - currentX;
        double dy = corner.position.y - currentY;
        return Math.sqrt(dx * dx + dy * dy) * 2.54; // returns distance in cm
    }

    public void setVelocityAndAngleByDist(Pose2d currPos, Pose2d corner){
        Pose2d offsetCorner = drive.getCornerOffsetVelocity(corner);
        double velocity = VELOCITY_SLOPE * getDistanceToTarget(currPos, offsetCorner) + VELOCITY_INTERCEPT;

//        double velocity = VELOCITY_SLOPE * getDistanceToTarget(currPos, corner) + VELOCITY_INTERCEPT;
        double angle = getCurrentVelocity() * HOOD_SLOPE + HOOD_INTERCEPT;

        // No longer in zone? You say so?! Stop wasting energy then!!! - R
//        enabledVel.set(isInsideTheZone(currPos).get());
        enabledVel.set(true);


        setTargetVelocity(velocity);
        setHoodAngle(angle);
    }

    public Command VelocityUpdate() {
        return Command.builder()
                .update(() -> {
                    telemetry.addData("hood angle", servoHood.getPosition());
                    if (isAboutToShot.get()) setVelocityAndAngleByDist(drive.getPose().value(), goal);
                    else {
                        setTargetVelocity(1000);
                        setHoodAngle(HOOD_MIN_POSITION);
                    }
                })
                .requires(this)
                .build();
    }

    public Command WaitForRPM(double maxMilliseconds) {
        ElapsedTime timer = new ElapsedTime();
        return Command.builder()
                .init(timer::reset)
                .update(() -> {
                    double error = targetVelocity - getCurrentVelocity();
                    telemetry.addData("Velocity Error", error);
                })
                .finished(() -> {
                    double error = Math.abs(targetVelocity - getCurrentVelocity());
                    return error <= RPM_TOLERANCE || timer.milliseconds() > maxMilliseconds;
                })
                .build();
    }

    public Command update() {
            return Command.builder()
                    .update(() -> {
                        if (enabledVel.get()) {
                            double currentVelocity = getCurrentVelocity(); //RPM
                            double power = flywheelPID.update(targetVelocity, currentVelocity) + flywheelFeedforward.update(targetVelocity, 0);
                            power = power * (Robot.nominalVoltage / voltageSensor.getVoltage());

                            shooter1.setPower(power);
                            shooter2.setPower(power);
                        }
                        else
                        {
                            shooter1.setPower(0);
                            shooter2.setPower(0);
                        }
                        double error = Math.abs(targetVelocity - getCurrentVelocity());
                        telemetry.addData("Velocity Error", error);
                    })
                    .requires(this)
                    .build();
    }

    public Command reset() {
        return new SequentialCommand(
                new InstantCommand(() -> {
                    servoHood.setPosition(HOOD_MIN_POSITION);
                    blockShooter();
                    enabledVel.set(true);
                    isAboutToShot.set(false);
                })
        );
    }

//    @Override
//    public SubsystemFlavor flavor() {
//        return SubsystemFlavor.ExpansionHubOnly;
//    }
}