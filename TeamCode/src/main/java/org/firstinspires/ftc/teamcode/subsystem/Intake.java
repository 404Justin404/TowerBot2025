package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;

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
        return new InstantCommand(() -> intakeMotor.setPower(0.20*Robot.nominalVoltage/voltageSensor.getVoltage()));
    }

    public Command stop() {
        return new InstantCommand(() -> intakeMotor.setPower(0));
    }

//    @Override
//    public SubsystemFlavor flavor() {
//        return SubsystemFlavor.ExpansionHubOnly;
//    }
}