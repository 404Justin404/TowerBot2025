package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.ftc.localization.localizers.ThreeWheelIMULocalizer;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.oraclelocalizer.SmartLocalizer;

/** This is the FollowerBuilder.
 * It is used to create Followers with a specific drivetrain + localizer without having to use a full constructor
 *
 * @author Baron Henderson - 20077 The Indubitables
 */
public class OracleFollowerBuilder {
    private final FollowerConstants constants;
    private PathConstraints constraints;
    private final HardwareMap hardwareMap;
    private Localizer localizer;
    private Drivetrain drivetrain;

    public OracleFollowerBuilder(FollowerConstants constants, HardwareMap hardwareMap) {
        this.constants = constants;
        this.hardwareMap = hardwareMap;
        constraints = PathConstraints.defaultConstraints;
    }

    public OracleFollowerBuilder setLocalizer(Localizer localizer) {
        this.localizer = localizer;
        return this;
    }

    public OracleFollowerBuilder setDrivetrain(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        return this;
    }

    // First do pedro tests with only Pinpoint and only THEN you can start implementing SmartLocalizer
    public OracleFollowerBuilder pinpointLocalizer(PinpointConstants lConstants) {
        return setLocalizer(new PinpointLocalizer(hardwareMap, lConstants));
    }

    public OracleFollowerBuilder smartLocalizer(SmartLocalizerConstants lConstants) {
        return setLocalizer(new SmartLocalizerPedro(hardwareMap, lConstants));
    }

    public OracleFollowerBuilder mecanumDrivetrain(MecanumConstants mecanumConstants) {
        return setDrivetrain(new Mecanum(hardwareMap, mecanumConstants));
    }

    public OracleFollowerBuilder pathConstraints(PathConstraints pathConstraints) {
        this.constraints = pathConstraints;
        PathConstraints.setDefaultConstraints(pathConstraints);
        return this;
    }

    public Follower build() {
        return new Follower(constants, localizer, drivetrain, constraints);
    }
}
