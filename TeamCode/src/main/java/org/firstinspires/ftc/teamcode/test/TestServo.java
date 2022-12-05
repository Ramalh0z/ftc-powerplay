package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.DestemidosHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.GripSystem;

@TeleOp(name = "TestServo", group = "Tests")
public class TestServo extends LinearOpMode {
    private DestemidosHardware robot;
    private ServoControllerEx servoController;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new DestemidosHardware(hardwareMap);
        servoController = (ServoControllerEx) robot.servoMão.getController();

        waitForStart();
        while (opModeIsActive()) {
            if(isStopRequested()) return;

            ArmSystem.movimentarBraço(gamepad2, robot);

            GripSystem.coletarCones(gamepad2, robot);

            debugServoInfo(robot.servoMão, telemetry);
            telemetry.addData("Servo PWM Range: ",
                    servoController.getServoPwmRange(robot.servoMão.getPortNumber())
            );
            telemetry.addData("Servo Connection Info: ",
                    servoController.getConnectionInfo()
            );

            //debugServoInfo(robot.servoGarraA, telemetry);
            //debugServoInfo(robot.servoGarraB, telemetry);
        }
    }


    void debugServoInfo(Servo servo, Telemetry telemetry) {
        telemetry.addData("Servo Name:", servo.getDeviceName());
        telemetry.addData("Servo Position:", servo.getPosition());
        telemetry.addData("Servo Direction:", servo.getDirection());
        telemetry.addData("Servo Port:", servo.getPortNumber());
    }
}