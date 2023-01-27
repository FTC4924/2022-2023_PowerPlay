package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="NewTeleop")
public class NewRedTeleop extends NewTeleopBase {
    @Override
    protected AllianceColor getAlliance() {
        return AllianceColor.RED;
    }
}
