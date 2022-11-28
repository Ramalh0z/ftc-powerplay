package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class AutonomousModule {

    // baseado no código dos controles
    protected void SimularControleReal(DcMotorEx[] motores, double inputY, double inputX, double inputGiro) {

        // modificadores essenciais pro movimento
        final double porcentagem_força = 0.80;

        double joystick_y = -inputY    * porcentagem_força;
        double joystick_x = inputX     * porcentagem_força;
        double giro       = inputGiro  * porcentagem_força;

        motores[0].setPower(joystick_y + joystick_x + giro); // Direita-Frente
        motores[1].setPower(joystick_y - joystick_x + giro); // Direita-Trás
        motores[2].setPower(joystick_y - joystick_x - giro); // Esquerda-Frente
        motores[3].setPower(joystick_y + joystick_x - giro); // Esquerda-Trás
    }

    // OBS: toda vez que finalizamos uma ação no robô, ZERAMOS o motor por conveniência

    // MOTORES
    public void ZerarMotores(DcMotorEx[] motores) {
        for (DcMotor motor : motores) {
            motor.setPower(0);
        }
    }

    // frente
    public void MoverFrente(DcMotorEx[] motors, double power) {
        SimularControleReal(motors, -power, 0, 0);
    }

    // trás
    public void MoverTras(DcMotorEx[] motors, double power) {
        SimularControleReal(motors, power, 0, 0);
    }

    // Mover Direita
    public void MoverDireita(DcMotorEx[] motors, double power) {
        SimularControleReal(motors, 0, 0, power);
    }

    // Mover Esquerda
    public void MoverEsquerda(DcMotorEx[] motors, double power) {
        SimularControleReal(motors, 0, 0, -power);
    }

    // Rotacionar Direita
    public void RotacionarDireita(DcMotorEx[] motors, double power) {
        SimularControleReal(motors, 0, power, 0);
    }

    // Rotacionar Esquerda
    public void RotacionarEsquerda(DcMotorEx[] motors, double power) {
        SimularControleReal(motors, 0, -power, 0);
    }

    // ATUADORES
    public void MoverBraço(DcMotorEx[] atuadores, double power) {
        atuadores[0].setPower(-power);
        atuadores[1].setPower(-power);
    }

    public void SugarObjeto(DcMotorEx garra, double power) {
        garra.setPower(-power);
    }

    public void SoltarObjeto(DcMotorEx garra, double power) {
        garra.setPower(power);
    }

    public void LigarMotorPato(DcMotorEx motor_pato) {
        motor_pato.setPower(-0.8F);
    }
}