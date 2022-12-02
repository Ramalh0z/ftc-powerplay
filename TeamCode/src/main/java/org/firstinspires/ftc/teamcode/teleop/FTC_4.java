package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DestemidosHardware;
import org.firstinspires.ftc.teamcode.hardware.RobotConfiguration;
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

        waitForStart();
        while (opModeIsActive()) {
            for (LynxModule hub : hubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }

            // movimentação padrão das partidas
            MovementSystem.controleOmnidirecionalClassico(gamepad1, robot);

            // controles do braço e da mão
            robot.motorCentro.setPower(-gamepad2.right_stick_x * RobotConfiguration.usoDoMotorCentro);

            double controlPower = gamepad2.left_stick_y * RobotConfiguration.usoDasGarras;
            robot.motorBraçoA.setPower(controlPower);
            robot.motorBraçoB.setPower(controlPower);

            GripSystem.coletarCones(gamepad2, robot);
        }
    }
}