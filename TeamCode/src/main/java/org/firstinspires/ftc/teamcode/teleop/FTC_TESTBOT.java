package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MovementSystem;
import org.firstinspires.ftc.teamcode.hardware.DestemidosHardware;
import org.firstinspires.ftc.teamcode.utils.RobotLogger;

import java.util.List;

@TeleOp(name="TESTBOT", group = "Test")
public class FTC_TESTBOT extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // setup the hardware components
        DestemidosHardware robot = new DestemidosHardware(hardwareMap);
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while(opModeIsActive()) {
            for (LynxModule hub: hubs){
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }

            double joystick_y  = -gamepad1.left_stick_y;
            double joystick_x  = gamepad1.left_stick_x;
            double giro        = gamepad1.right_stick_x;

            // o ângulo e o módulo do vetor gerados com o joystick
            double theta = Math.atan2(joystick_y, joystick_x);
            double direction = Math.hypot(joystick_x, joystick_y);

            MovementSystem.controleMecanumAvançado(theta, direction, giro, robot);

            // Debug de informações
            RobotLogger.debugRodasInfo(telemetry, robot);
            RobotLogger.debugControles(telemetry, gamepad1, gamepad2);
            telemetry.update();
        }
    }
}