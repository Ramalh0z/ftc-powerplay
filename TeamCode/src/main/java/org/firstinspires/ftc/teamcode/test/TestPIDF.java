package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.DestemidosHardware;


@Autonomous(name = "TestPIDF", group = "Test")
@Disabled
public class TestPIDF extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DestemidosHardware robot = new DestemidosHardware(hardwareMap);


        waitForStart();

    }
}
