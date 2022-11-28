package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * RobotLogger - Responsável por transmitir informações dos
 * devidos dispositivos para o Telemetry, como forma de ajudar
 * a identificar possiveis problemas na hora de testatgem ou debug
 * dos OpModes.
 */

public final class RobotLogger {

    // Processa uma lista de N Motores, informando apenas o seu nome + força atual 
    public static void debugMotorPower(Telemetry telemetry, DcMotorEx[] motors) {
        telemetry.addLine("\nMOTORES - Força:");
        for (DcMotorEx motor : motors) {
            telemetry.addData("Força do motor " + motor.getDeviceName() + " : ", motor.getPower());
        }
    }

    // Processa uma lista de N Motores, informando apenas o seu nome + velocidade do encoder atual
    public static void debugMotorVelocity(Telemetry telemetry, DcMotorEx[] motors) {
        telemetry.addLine("\nMOTORES - Velocidade:");
        for (DcMotorEx motor : motors) {
            telemetry.addData("Velocidade do motor " + motor.getDeviceName() + " : ", motor.getVelocity());
        }
    }

    // Processa uma lista de N Motores, informando apenas o seu nome + posição atual
    public static void debugMotorPosition(Telemetry telemetry, DcMotorEx[] motors) {
        telemetry.addLine("\nMOTORES:");
        for (DcMotorEx motor : motors) {
            telemetry.addData("Posição atual do motor " + motor.getDeviceName() + " : ", motor.getCurrentPosition());
        }
    }

    // Processa uma lista de N Motores, informando apenas o seu nome + corrente atual
    public static void debugMotorCurrent(Telemetry telemetry, DcMotor[] motors) {
        telemetry.addLine("\nMOTORES:");
        for (DcMotor motor : motors) {
            telemetry.addData("Corrente do motor " + motor.getDeviceName() + " : ", motor.getPower());
        }
    }

    // Processa uma lista de N Motores, informando apenas o seu nome + coeficientes PIDF atuais
    public static void debugMotorPIDF(Telemetry telemetry, DcMotorEx[] motors) {
        telemetry.addLine("\nMOTORES:");
        for (DcMotorEx motor : motors) {
            telemetry.addData("Coeficiente do motor " + motor.getDeviceName() + " : ", motor.getPIDFCoefficients(motor.getMode()));
        }
    }

    // Invertemos o valor do Y, por causa que o SDK define
    // para cima como "-1.0", e para baixo como "+1.0"
    public static void debugControles(Telemetry telemetry, Gamepad controller1, Gamepad controller2) {
        telemetry.addLine("\nGAMEPADS: ");
        telemetry.addData("Gamepad1:", "Y: %.2f  X: %.2f  Giro: %.2f", controller1.left_stick_y, controller1.left_stick_x, controller1.right_stick_x);
        telemetry.addData("Gamepad2:", "Y: %.2f  X: %.2f  Giro: %.2f", controller2.left_stick_y, controller2.left_stick_x, controller2.right_stick_x);
    }
}
