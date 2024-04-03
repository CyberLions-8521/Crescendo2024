/**
 * Code taken from https://github.com/REVrobotics/MAXSwerve-Java-Template
Copyright (c) 2018-2023 REV Robotics

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. Neither the name of REV Robotics nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
 */
package frc.robot.Util;
import edu.wpi.first.math.MathUtil;

public class SwerveUtils {

    /**
     * Steps a value towards a target with a specified step size.
     * @param _current The current or starting value.  Can be positive or negative.
     * @param _target The target value the algorithm will step towards.  Can be positive or negative.
     * @param _stepsize The maximum step size that can be taken.
     * @return The new value for {@code _current} after performing the specified step towards the specified target.
     */
    public static double StepTowards(double _current, double _target, double _stepsize) {
        if (Math.abs(_current - _target) <= _stepsize) {
            return _target;
        }
        else if (_target < _current) {
            return _current - _stepsize;
        }
        else {
            return _current + _stepsize;
        }
    }

    /**
     * Steps a value (angle) towards a target (angle) taking the shortest path with a specified step size.
     * @param _current The current or starting angle (in radians).  Can lie outside the 0 to 2*PI range.
     * @param _target The target angle (in radians) the algorithm will step towards.  Can lie outside the 0 to 2*PI range.
     * @param _stepsize The maximum step size that can be taken (in radians).
     * @return The new angle (in radians) for {@code _current} after performing the specified step towards the specified target.
     * This value will always lie in the range 0 to 2*PI (exclusive).
     */
    public static double StepTowardsCircular(double _current, double _target, double _stepsize) {
        _current = WrapAngle(_current);
        _target = WrapAngle(_target);

        double stepDirection = Math.signum(_target - _current);
        double difference = Math.abs(_current - _target);
        
        if (difference <= _stepsize) {
            return _target;
        }
        else if (difference > Math.PI) { //does the system need to wrap over eventually?
            //handle the special case where you can reach the target in one step while also wrapping
            if (_current + 2*Math.PI - _target < _stepsize || _target + 2*Math.PI - _current < _stepsize) {
                return _target;
            }
            else {
                return WrapAngle(_current - stepDirection * _stepsize); //this will handle wrapping gracefully
            }

        }
        else {
            return _current + stepDirection * _stepsize;
        }
    }

    /**
     * Finds the (unsigned) minimum difference between two angles including calculating across 0.
     * @param _angleA An angle (in radians).
     * @param _angleB An angle (in radians).
     * @return The (unsigned) minimum difference between the two angles (in radians).
     */
    public static double AngleDifference(double _angleA, double _angleB) {
        double difference = Math.abs(_angleA - _angleB);
        return difference > Math.PI? (2 * Math.PI) - difference : difference;
    }

    /**
     * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
     * @param _angle The angle (in radians) to wrap.  Can be positive or negative and can lie multiple wraps outside the output range.
     * @return An angle (in radians) from 0 and 2*PI (exclusive).
     */
    public static double WrapAngle(double _angle) {
        return MathUtil.inputModulus( _angle,0, Math.PI * 2);

    }
}
