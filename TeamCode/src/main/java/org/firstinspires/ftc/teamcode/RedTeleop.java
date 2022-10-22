package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RedTeleop")
public class RedTeleop extends TeleopBase {
    protected AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }
}