package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.controllers.PIDController;
import org.firstinspires.ftc.teamcode.hardware.DestemidosHardware;


@Autonomous(name = "TestPIDF", group = "Test")
@Disabled
public class TestPIDF extends LinearOpMode {
    private DestemidosHardware robot;
    private PIDController controller;

    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new DestemidosHardware(hardwareMap);
        controller = new PIDController(kP, kI, kD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        // (ramalho) é bem provável que isso não funcione, mas ok
        double posição_roda = robot.motorDireitaFrente.getCurrentPosition();

        double nova_posição = controller.update(10, posição_roda);

        robot.motorDireitaFrente.setPower(nova_posição);
    }
}
