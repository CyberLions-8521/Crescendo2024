package frc.robot.Util;

/** Exponential Drive functionality for joysticks
 * See sample post here for context
 * https://www.vexforum.com/t/guide-to-exponential-drive-function/58590
 * 
 * or another one here
 * https://mathmaine.com/2014/01/20/polynomials-and-vex-drive-motor-control/
 */


public class mathProfiles {
    /** Equation here is piecewise function f, where f(x) = a^x-1, when 0 <= x <= 1,
    * and f(x) = -a^(-x)+1 when -1 <= x < 0.
    *
    * @param    x   The input value which serves as the exponent; it must be a value between -1 and 1 inclusive
    * @param    a   The base
    * @return   The resulting value after the input value has been scaled using the exponential equation
    */
    // public static double exponentialDrive(double x, double a) {
    //     if (x >= 0 && x <= 1) {
    //         return Math.pow(a, x) - 1;
    //     } else if (x >= -1 && x <= 0) {
    //         return -Math.pow(1/a, x) + 1;
    //     } else {
    //         throw new IllegalArgumentException("Input x is outside the valid range (-1 <= x <= 1)");
    //     }
    // }
    public static double exponentialDrive(final double x, final double a) {
        if (Math.abs(x) > 1) {
            throw new IllegalArgumentException("Input x is outside the valid range [-1, 1]");
        }

        if (x >= 0) {
            return Math.pow(a, x) - 1.0;
        }
        return -Math.pow(a, -x) + 1.0;
    }

    /** Testing other mathematical functions that provide equivalent functionality 
    * Use exponentialDrive() for best results
    */
    public static double tanDrive(double x) {
        if (x >= -1 && x <= 1) {
            return Math.tan(x);
        } else {
            throw new IllegalArgumentException("Input x is outside the valid range (-1 <= x <= 1)");
        }
    }

    public static double sigmoidDrive(double x, double steepness) {
        // Adjust the constant term for starting at (0, 0)
        return (1 / (1 + Math.exp(-steepness * x))) - 0.5;
      }
    public static double tanhDrive(double x, double a){
        return a * Math.tanh( x);
    }
}
  