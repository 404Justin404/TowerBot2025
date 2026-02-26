package org.firstinspires.ftc.teamcode.subsystem;
//revizuire dupa completion
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;

import java.util.List;

@Config
public class Robot {
    public static double nominalVoltage=10.0;
    private final OpMode opMode;
    private final boolean color;
    public final Turret flywheel;
    public final MecanumDrive drive;
    public final Intake intake;
    public final Turret turret;
    public final Lift lift;

    List<LynxModule> lynxModules;

    public Robot(OpMode mode,boolean color)
    {
        this.opMode = mode;
        OracleLynxVoltageSensor voltageSensor = mode.hardwareMap.getAll(OracleLynxVoltageSensor.class).iterator().next();
        voltageSensor.setPolicy(OracleLynxVoltageSensor.OracleLynxVoltageSensorPolicy.CACHED);
        voltageSensor.setVoltageCacheFreshness(50);
        this.flywheel=new Turret(mode);
        this.intake = new Intake(mode);
        this.drive = new MecanumDrive(mode.hardwareMap, opMode.telemetry);
        this.turret = new Turret(mode);
        this.lift=new Lift(mode);
        this.color = color;

        lynxModules = opMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    public void read()
    {
        for (LynxModule lynxModule : lynxModules) {
            lynxModule.clearBulkCache();
            lynxModule.getBulkData();
        }
    }

    public Command reset()
    {
        return new SequentialCommand(
                new ParallelCommand(
                        turret.reset(),
                        intake.idleIntake()
                )
        );
    }

    public Command update()
    {
        return new ParallelCommand(
                drive.update(),
                turret.update()
        );
    }
}
