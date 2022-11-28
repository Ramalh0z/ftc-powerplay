package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.MovementModule;
import org.firstinspires.ftc.teamcode.robots.BaseRobot;
import org.firstinspires.ftc.teamcode.utils.RobotLogger;

import java.util.List;

@TeleOp(name="TESTBOT", group = "Test")
public class FTC_TESTBOT extends LinearOpMode {

    // módulos
    private final BaseRobot robot = new BaseRobot();
    private List<LynxModule> hubs;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.inicializarHardware(hardwareMap);
        hubs = hardwareMap.getAll(LynxModule.class);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while(opModeIsActive()) {
            for (LynxModule hub: hubs){
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
            // movimentos principais
            MovementModule.controleOmnidirecional(gamepad1, robot.motores, 1.0);

            // váriaveis do braço


            // Debug de informações
            RobotLogger.debugMotorPower(telemetry, robot.motores);
            RobotLogger.debugMotorVelocity(telemetry, robot.motores);
            RobotLogger.debugMotorPIDF(telemetry, robot.motores);
            telemetry.update();
        }
    }
}