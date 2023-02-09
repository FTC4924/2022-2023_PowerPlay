package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name="BlueTeleop")
public class BlueTeleop extends TeleopBase {
    protected AllianceColor getAllianceColor() {
        return AllianceColor.BLUE;
    }

}
