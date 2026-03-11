package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

public class Intake extends Subsystem {

    private final DcMotorImplEx intakeMotor;
    private final OracleLynxVoltageSensor voltageSensor;

    public Intake(OpMode opMode) {
        super(opMode);

        intakeMotor = hardwareMap.get(DcMotorImplEx.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hardwareMap.getAll(OracleLynxVoltageSensor.class).iterator().next();
    }

    public Command intake() {
        return new InstantCommand(() -> intakeMotor.setPower(1*Robot.nominalVoltage/voltageSensor.getVoltage()));
    }

    public Command outake() {
        return new InstantCommand(() -> intakeMotor.setPower(-1*Robot.nominalVoltage/voltageSensor.getVoltage()));
    }

    public Command slowIntake() {
        return new InstantCommand(() -> intakeMotor.setPower(0.6*Robot.nominalVoltage/voltageSensor.getVoltage()));
    }

    public Command idleIntake() {
        return new InstantCommand(() -> intakeMotor.setPower(0.3*Robot.nominalVoltage/voltageSensor.getVoltage()));
    }

    public Command stop() {
        return new InstantCommand(() -> intakeMotor.setPower(0));
    }

    public double getCurrentAmps() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS) * Robot.nominalVoltage / voltageSensor.getVoltage();
    }
    public void BallNumber(double currentAmps) {
        if(currentAmps>=6)telemetry.addLine("Intake has 3 balls");
        else if(currentAmps>=4)telemetry.addLine("Intake has 2 balls");
        else if(currentAmps>1.7)telemetry.addLine("Intake has 1 ball");
        else telemetry.addLine("Intake has no balls");
    }
}

