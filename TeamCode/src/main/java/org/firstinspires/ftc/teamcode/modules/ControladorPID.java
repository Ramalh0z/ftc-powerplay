package org.firstinspires.ftc.teamcode.modules;

/*
 * Controlador PID - é uma espécie de módulo
 * que usamos temporáriamente para testar o uso
 * do PID no autônomo do robô
 * 
 * está bem longe de uma implementação relmente usável, porém
 * é apenas uma "prova de conceito", nada foi realmente usado
 * na competição
 */

public class ControladorPID {
    // Coeficientes com as indicações de davi
    public double kP;   // valores altos, davi recomendou usar 1.5 de inicio
    public double kI;   // valores MUITO baixos e de preferência negativo, exemplo: 0.0001
    public double kD;   // valores bem altos (100+), por que a correção da derivada é bem suave

    // usadas apenas pelo próprio controlador
    protected double SomaDaIntegral = 0;
    protected double lastErro = 0;

    public ControladorPID(double kp, double ki, double kd)
    {
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;
    }

    public double calcularPID(double referencial, double valorAtual)
    {
        double erro = referencial - valorAtual;

        // Proporcional
        double P = (erro * kP);

        // Integral
        SomaDaIntegral += erro;
        double I = SomaDaIntegral * kI;

        // Derivada
        double D = (erro - lastErro);
        lastErro = erro;

        return P + I + D;
    }
}
