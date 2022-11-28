package org.firstinspires.ftc.teamcode.utils;
 
/* Funções de Interpolação Linear
*
* a = valor inicial
* b = valor final
* t = porcentagem da interpolação ao longo de a até b
*
* iMin e iMax = input mínimo e máximo
* oMin e oMax = saída mínima e máxima
* value = valor entre o oMin e o oMax
*/
public final class MathUtils {

    // interpola os valores a depender de uma porcentagem (ou tempo)
    public static double Lerp(double min, double max, double t) {
        return (1 - t) * min + t * max;
    }

    // sendo o inverso da lerp, aqui retornamos a porcentagem/tempo
    public static double InverseLerp(double min, double max, double v) {
        return (v - min) / (max - min);
    }

    // remapeia os valores de acordo com os limites definidos a baixo
    public static double ReMap(double iMin, double iMax, double oMin, double oMax, double value) {
        double t = InverseLerp(iMin, iMax, value);
        return Lerp(oMin, oMax, t);
    }

    // uma interpolação bem mais suave e simples
    // baseado em: https://thebookofshaders.com/glossary/?search=smoothstep
    public static double SmoothStep(double min, double max, double value) {
        double x = Clamp(InverseLerp(min, max, value), 0.0, 1.0);
        return x * x * (3.0 - 2.0 * x);
    }

    // arredondamento bem comum
    public static double Clamp(double value, double min, double max) {

        if (value > max) return max;

        if (value < min) return min;

        return value;
    }
}
