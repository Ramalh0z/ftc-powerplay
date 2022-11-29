package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    // usadas apenas pelo próprio controlador
    private double SomaDaIntegral = 0;
    private double LimiteDaIntegra;
    private double lastError = 0;
    private ElapsedTime timer;

    // podemos usar
    public double kP;
    public double kI;
    public double kD;

    public PIDController(double kp, double ki, double kd)
    {
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;
    }

    public double update(double referencia, double valorAtual)
    {
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        double erro = referencia - valorAtual;

        // Proporcional
        double P = (erro * kP);

        // Integral
        SomaDaIntegral += erro * timer.milliseconds();
        double I = (SomaDaIntegral * kI);

        // Derivada
        double D = (erro - lastError) / timer.milliseconds();
        lastError = erro;

        return (P + I + D);
    }
}
