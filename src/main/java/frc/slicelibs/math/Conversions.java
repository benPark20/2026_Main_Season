package frc.slicelibs.math;

public class Conversions {

    /** Motor rotations -> wheel distance in meters */
    public static double talonToMeters(double motorRot, double circumference, double gearRatio) {
        return (motorRot / gearRatio) * circumference;
    }

    /** Wheel distance in meters -> motor rotations */
    public static double metersToTalon(double wheelMeters, double circumference, double gearRatio) {
        return (wheelMeters / circumference) * gearRatio;
    }

    /** Motor RPS -> wheel velocity in m/s */
    public static double talonToMPS(double motorRPS, double circumference, double gearRatio) {
        return (motorRPS / gearRatio) * circumference;
    }

    /** Wheel velocity in m/s -> motor RPS */
    public static double MPSToTalon(double wheelMPS, double circumference, double gearRatio) {
        return (wheelMPS / circumference) * gearRatio;
    }

    /** Motor RPS^2 -> wheel acceleration in m/s^2 */
    public static double talonToMPSSquared(double motorRPSSquared, double circumference, double gearRatio) {
        return (motorRPSSquared / gearRatio) * circumference;
    }

    /** Motor rotations -> mechanism angle in degrees */
    public static double talonToDegrees(double motorRot, double gearRatio) {
        return (motorRot / gearRatio) * 360.0;
    }

    /** Mechanism angle in degrees -> motor rotations */
    public static double degreesToTalon(double mechDeg, double gearRatio) {
        return (mechDeg / 360.0) * gearRatio;
    }

    /** Motor RPS -> mechanism RPM */
    public static double talonToRPM(double motorRPS, double gearRatio) {
        return (motorRPS / gearRatio) * 60.0;
    }

    /** Mechanism RPM -> motor RPS */
    public static double RPMToTalon(double mechRPM, double gearRatio) {
        return (mechRPM * gearRatio) / 60.0;
    }

    /** Mechanism RPM + wheel circumference -> m/s */
    public static double RPMToMPS(double mechRPM, double circumference) {
        return mechRPM * circumference / 60.0;
    }
}