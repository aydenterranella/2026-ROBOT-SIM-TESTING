package frc.firecontrol;

//One-time LUT generator. Run (Run as -> Java Application),
 
public class GenerateLUT {

  public static void main(String[] args) {
    var params = new ProjectileSimulator.SimParameters(
        0.215,   // ball mass kg       — game manual 5.10.1
        0.1501,  // ball diameter m    — game manual 5.10.1
        0.47,    // drag coeff  — smooth sphere, don't change
        0.2,     // Magnus coeff       — don't change
        1.225,   // air density kg/m³  — don't change
        0.5715,    //EXIT HEIGHT (m)    — floor → ball exit point, replace with CAD value
        0.0762,  // WHEEL DIAMETER (m) — measure flywheel with calipers, replace with actual
        1.83,    //— hopper opening height
        0.6,     // slip factor — good starting point, tune on robot later
        0.0,     // fixed angle — unused (using variable angle sweep below)
        0.001,   // timestep
        1500, 6000, 25, 5.0
    );

    // Hood angle range in degrees
    double minHoodDeg =  1.8;
    double maxHoodDeg = 54.0; 

    var sim = new ProjectileSimulator(params);
    ShotLUT lut = sim.generateVariableAngleShotLUT(minHoodDeg, maxHoodDeg, 0);

    // Distances match the existing TurretConstants entries
    double[] distances = { 2.5, 3.0, 3.5, 4.1, 4.9 };

  
    for (double d : distances) {
      ShotParameters p = lut.get(d);
      System.out.printf("    { %.1f, %.4f },%n", d, p.rps() / 60.0); // RPM -> RPS
    }
    System.out.println("};");

    System.out.println("\npublic static final double[][] HOOD_ANGLES = {");
    for (double d : distances) {
      ShotParameters p = lut.get(d);
      // degrees → rotations (motor uses rotations)
      System.out.printf("    { %.1f, %.5f },  // %.1f deg%n",
          d, p.angleDeg() / 360.0, p.angleDeg());
    }
    System.out.println("};");

    System.out.println("\npublic static final double[][] SHOOTER_TOF = {");
    for (double d : distances) {
      ShotParameters p = lut.get(d);
      System.out.printf("    { %.1f, %.3f },%n", d, p.tofSec());
    }
    System.out.println("};");
  }
}
