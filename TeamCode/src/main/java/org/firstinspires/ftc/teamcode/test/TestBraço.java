package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DestemidosHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;


@TeleOp(name = "Test Braço", group = "Test")
@Disabled
public class TestBraço extends LinearOpMode {
    private DestemidosHardware robo;

    @Override
    public void runOpMode() throws InterruptedException {
        robo = new DestemidosHardware(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            ArmSystem.movimentarBraço(gamepad2, robo);
        }
    }

}
