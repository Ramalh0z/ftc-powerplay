package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Deprecated
final class OdometryModule {
    /* 
     * ODOMETRIA: é a técnica utilizada para medir distâncias algum dispositivo (no nosso caso, os encoders)
     *
     * Através deste sistema, é possivel realizar movimentos precisos e flúidos com uma margem
     * de erro bastante tolerável, mas para isto ocorrer precisamos realizar algumas modificações e
     * cálculos estrátegicos em relação ao nosso robô.
     *
     * De inicio precisamos das constantes (a maioria em centímetros) abaixo:
     */

    // Constantes
    final static double DISTANCIA_VERTICAL_MOTORES   = 29.0;    // intervalo entre os motores que são paralelos
    final static double DISTANCIA_HORIZONTAL_MOTORES = 42.2;    // intervalo dos motores que estão alinhados
    final static double RAIO_RODA = 3.75;
    final static double LEITURAS_POR_REVOLUCAO = 336.0;         // 28 leituras por padrão * 12 (acúumulo total de redução das caixas 3:1 e 4:1)

    // medidas úteis
    final static double COMPRIMENTO_RODA = 2.0 * Math.PI * RAIO_RODA;
    final static double DV = DISTANCIA_VERTICAL_MOTORES / 2.0;
    final static double DH = DISTANCIA_HORIZONTAL_MOTORES / 2.0;
    final static double CM_POR_TICK = COMPRIMENTO_RODA / LEITURAS_POR_REVOLUCAO;

    final static double velocidade = 1000.0; // nós modificamos essa aqui

    // array dos valores de encoders que serão lidos/escritos ao longo da execução do robô
    double[] encoders         = new double[4];
    double[] ultimosEncoders  = new double[4];
    double[] deltaDosEncoders = new double[4];

    // valores do sensor IMU
    double posicaoX = 0.0;
    double ultimaPosicaoX = 0.0;
    double deltaPosicaoX = 0.0;

    public void odometria2(DcMotorEx[] motores)
    {
        // OBS: temos que contar com a distancia dos motores em relação ao centro
        // então dividi em 2 grupos: grupo frente e grupo trás

        // grupo frente = DF e EF
        encoders[0] = motores[0].getCurrentPosition();
        encoders[1] = motores[1].getCurrentPosition();

        // grupo trás = DT e ET
        encoders[2] = motores[2].getCurrentPosition();
        encoders[3] = motores[3].getCurrentPosition();

        // calcular o intervalo em relação ao ultimo valor medido pelos encoders:

        // grupo frente
        deltaDosEncoders[0] = encoders[0] - ultimosEncoders[0];
        deltaDosEncoders[1] = encoders[1] - ultimosEncoders[1];

        // grupo trás
        deltaDosEncoders[2] = encoders[2] - ultimosEncoders[2];
        deltaDosEncoders[3] = encoders[3] - ultimosEncoders[3];

        // posição relativa do robo
        //deltaPosicaoX = poseFinal.x - poseInicial.x;

        // estimamos a orientação do robô neste deltaTheta
        double deltaTheta = CM_POR_TICK * (deltaDosEncoders[0] - deltaDosEncoders[1]) / DISTANCIA_HORIZONTAL_MOTORES;

        // Como não temos um odõmetro perpendicular, futuramente usaremos a posição do eixo X
        // dada pelo sensor IMU embutido no controlhub

        // estimamos a coordenada X, para o par de motores da frente e tbm os de trás
        double deltaX = CM_POR_TICK * (deltaDosEncoders[1] + deltaDosEncoders[0]) / 2.0;

        // OBS: pela diferença na posição dos motores no eixo Y em relação ao centro
        // temos essas soma e subtrção do DH

        // grupo frente
        double deltaYFrente = CM_POR_TICK * ((deltaDosEncoders[0] - deltaDosEncoders[1]) + deltaPosicaoX) * DH / DISTANCIA_HORIZONTAL_MOTORES;

        // grupo trás
        double deltaYTrás = CM_POR_TICK * ((deltaDosEncoders[0] - deltaDosEncoders[1]) + deltaPosicaoX) * DH / DISTANCIA_HORIZONTAL_MOTORES;

        // theta ao longo do movimento:
        //double theta = poseInicial.theta + (deltaTheta / 2.0);

        // poses da frente e de trás:
        //Pose2D targetPoseFrente = new Pose2D(0.0, 0.0, 0.0);
        //targetPoseFrente.theta += deltaTheta;
        //targetPoseFrente.x += deltaX * Math.cos(theta) - deltaYFrente * Math.sin(theta);
        //targetPoseFrente.y += deltaX * Math.sin(theta) + deltaYFrente * Math.cos(theta);

        //Pose2D targetPoseTras = new Pose2D(0.0,0.0,0.0);
        //targetPoseTras.theta += deltaTheta;
        //targetPoseTras.x += deltaX * Math.cos(theta) - deltaYTrás * Math.sin(theta);
        //targetPoseTras.y += deltaX * Math.sin(theta) + deltaYTrás * Math.cos(theta);

        // fim???
        //double verticalFrente = (targetPoseFrente.y * velocidade);
        //double verticalTras = (targetPoseTras.y * velocidade);
        //double lateral = (targetPoseFrente.x * velocidade);
        //double giro = (targetPoseFrente.theta * velocidade);

        // grupo frente
        //motores[0].setVelocity(verticalFrente + lateral + giro);
        //motores[1].setVelocity(verticalFrente - lateral - giro);

        // grupo trás
        //motores[2].setVelocity(verticalTras + lateral - giro);
        //motores[3].setVelocity(verticalTras - lateral + giro);

        // salva os ultimos valores
        ultimosEncoders[0] = encoders[0];
        ultimosEncoders[1] = encoders[1];
        ultimosEncoders[2] = encoders[2];
        ultimosEncoders[3] = encoders[3];
    }

    public void receberInputs(DcMotorEx[] motores, Position position)
    {
        // OBS: temos que contar com a distancia dos motores em relação ao centro
        // então dividi em 2 grupos: grupo frente e grupo trás

        // grupo frente = DF e EF
        encoders[0] = motores[0].getCurrentPosition();
        encoders[1] = motores[1].getCurrentPosition();

        // grupo trás = DT e ET
        encoders[2] = motores[2].getCurrentPosition();
        encoders[3] = motores[3].getCurrentPosition();

        // calcular o intervalo em relação ao ultimo valor medido pelos encoders:

        // grupo frente
        deltaDosEncoders[0] = encoders[0] - ultimosEncoders[0];
        deltaDosEncoders[1] = encoders[1] - ultimosEncoders[1];

        // grupo trás
        deltaDosEncoders[2] = encoders[2] - ultimosEncoders[2];
        deltaDosEncoders[3] = encoders[3] - ultimosEncoders[3];

        // posição relativa do robo
        posicaoX = position.x;

        // intervalo do eixo X
        deltaPosicaoX = posicaoX - ultimaPosicaoX;
    }

    public void moverComOdometria(DcMotorEx[] motores) {

        // TODO: testar a conversão de ticks (leituras) para centimetros
        // calcula os arcos que serão percorridos

        // grupo frente
        double ArcMotorDF = CM_POR_TICK * deltaDosEncoders[0];
        double ArcMotorEF = CM_POR_TICK * deltaDosEncoders[1];

        double ArcMotorDT = CM_POR_TICK * deltaDosEncoders[2];
        double ArcMotorET = CM_POR_TICK * deltaDosEncoders[3];

        // por enquanto vou comentar
        //double ArcCentro =  raio * poseAlvo.theta;

        // estimamos a orientação do robô neste deltaTheta
        double deltaTheta = (ArcMotorEF - ArcMotorDF) / DISTANCIA_HORIZONTAL_MOTORES;

        // fórmula do meio angulo
        double meioAnguloTheta = Math.sin(deltaTheta / 2);

        /* Usamos a lei dos cossenos simplificada, para calcular a posição no eixo Y:
         *
         * fórmula -> c^2 = a^2 + b^2 - 2*b*a * cos(C)
         *
         * nossa versão pura   -> y^2 = r^2 + r^2 - 2*r^2 * cos(theta)
         * versão simplificada -> y = 2*r * ( raizQuadrada(1- cos(theta) / 2)
         *
         * essa "raizQuadrada(1- cos(theta) / 2" pode ser trocada pelo seno do meio angulo
         *
         * por fim, o cálculo fica assim:
         */

        // OBS: pela diferença na posição dos motores no eixo Y em relação ao centro
        // temos essas soma e subtrção do DH

        // grupo frente
        double deltaYFrente = (ArcMotorDF + DH) * (meioAnguloTheta);

        // grupo trás
        double deltaYTrás = (ArcMotorEF - DH) * (meioAnguloTheta);

        // Como não temos um odõmetro perpendicular, futuramente usaremos a posição do eixo X
        // dada pelo sensor IMU embutido no controlhub

        // estimamos a coordenada X, para o par de motores da frente e tbm os de trás
        double deltaXFrente = 2 * (deltaPosicaoX / 2.0 + DV) * meioAnguloTheta;

        //Pose2D targetPoseFrente = new Pose2D(0.0, 0.0, 0.0);
        //targetPoseFrente.theta += deltaTheta;
        //targetPoseFrente.x += deltaXFrente;
        //targetPoseFrente.y += deltaYFrente;

        //Pose2D targetPoseTras = new Pose2D(0.0,0.0,0.0);
        //targetPoseTras.theta += deltaTheta;
        //targetPoseTras.x += deltaXFrente; // talvez funcione??
        //targetPoseTras.y += deltaYTrás;

        // salva os ultimos valores
        ultimosEncoders[0] = encoders[0];
        ultimosEncoders[1] = encoders[1];
        ultimosEncoders[2] = encoders[2];
        ultimosEncoders[3] = encoders[3];
    }
}