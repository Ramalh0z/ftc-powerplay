package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DestemidosHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.GripSystem;
import org.firstinspires.ftc.teamcode.subsystems.MovementSystem;

import java.util.List;

@TeleOp(name = "FTC_4", group = "TeleOps")
public class FTC_4 extends LinearOpMode {
    private DestemidosHardware robot;
    private List<LynxModule> hubs;

    @Override
    public void runOpMode() {
        // setup the hardware components
        robot = new DestemidosHardware(hardwareMap);
        hubs = hardwareMap.getAll(LynxModule.class);

        hubs.get(0).setConstant(Color.MAGENTA);
        hubs.get(1).setConstant(Color.MAGENTA);

        waitForStart();
        while (opModeIsActive()) {
            for (LynxModule hub : hubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }

            // movimentação padrão das partidas
            MovementSystem.controleOmnidirecionalClassico(gamepad1, robot);

            // controles do braço e da mão
            ArmSystem.movimentarBraço(gamepad2, robot);

            GripSystem.coletarCones(gamepad2, robot);

            telemetry.addData("mão - posição:", robot.servoMão.getPosition());
            telemetry.addData("mão - direção:", robot.servoMão.getDirection());

            telemetry.addData("garraA - posição", robot.servoGarraA.getPosition());
            telemetry.addData("garraB - posição", robot.servoGarraB.getPosition());
            telemetry.update();
        }
    }
}