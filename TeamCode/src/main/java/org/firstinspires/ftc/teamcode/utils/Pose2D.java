package org.firstinspires.ftc.teamcode.utils;
/*
 * Pose2D - Representa um vetor de 2 dimensões + componente extra 
 * que registra o ângulo de orientação da atual "pose"
 */

public class Pose2D {
    public double x;
    public double y;
    public double theta;

    public Pose2D(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = Math.toRadians(theta);
    }
}
