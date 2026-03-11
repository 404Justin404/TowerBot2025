package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.hardware.subsystem.SubsystemFlavor;

public class Lift extends Subsystem {

    private final CRServo backLeftLift, backRightLift, frontLeftLift, frontRightLift;

    // Current power state
    private double currentPower = 0.0;

    public Lift(OpMode opMode) {
        super(opMode);

        backLeftLift = hardwareMap.get(CRServo.class, "backLeftLift");
        backRightLift = hardwareMap.get(CRServo.class, "backRightLift");
        frontLeftLift = hardwareMap.get(CRServo.class, "frontLeftLift");
        frontRightLift = hardwareMap.get(CRServo.class, "frontRightLift");


        frontRightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftLift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        currentPower = power;
        backLeftLift.setPower(power);
        backRightLift.setPower(power);
        frontLeftLift.setPower(power);
        frontRightLift.setPower(power);
    }


    public Command liftUp() {
        return new InstantCommand(() -> setPower(1.0));
    }


    public Command liftDown() {
        return new InstantCommand(() -> setPower(-1.0));
    }

    public Command liftUp(double power) {
        return new InstantCommand(() -> setPower(Math.abs(power)));
    }

    public Command liftDown(double power) {
        return new InstantCommand(() -> setPower(-Math.abs(power)));
    }

    public Command stop() {
        return new InstantCommand(() -> setPower(0.0));
    }


    public Command hold() {
        return new InstantCommand(() -> setPower(0.1));
    }


    public double getCurrentPower() {
        return currentPower;
    }

    public void setLeftPower(double power) {
        backLeftLift.setPower(power);
        frontLeftLift.setPower(power);
    }

    public void setRightPower(double power) {
        backRightLift.setPower(power);
        frontRightLift.setPower(power);
    }

    public void emergencyStop() {
        setPower(0.0);
    }

    @Override
    public SubsystemFlavor flavor() {
        return SubsystemFlavor.ExpansionHubOnly;
    }
}