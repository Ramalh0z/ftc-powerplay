package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.GripSystem;
import org.firstinspires.ftc.teamcode.subsystems.MovementSystem;
import org.firstinspires.ftc.teamcode.hardware.DestemidosHardware;
import org.firstinspires.ftc.teamcode.utils.RobotLogger;

import java.util.List;

@Disabled
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

            // movimentação padrão das partidas
            MovementSystem.controleOmnidirecionalClassico(gamepad1, robot);

            // controles do braço e da mão
            ArmSystem.movimentarBraço(gamepad2, robot);

            GripSystem.coletarCones(gamepad2, robot);

            // Debug de informações
            RobotLogger.debugRodasInfo(telemetry, robot);
            RobotLogger.debugControles(telemetry, gamepad1, gamepad2);
            telemetry.update();
        }
    }
}