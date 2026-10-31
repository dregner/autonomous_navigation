#ifndef AUTONOMOUS_NAVIGATION_PID_HPP
#define AUTONOMOUS_NAVIGATION_PID_HPP

// Header-only, ROS-free PID helpers so the control math can be unit-tested
// without a simulator (see test/test_pid.cpp).

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace nav_ctrl
{

struct PidGains
{
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
    double i_max{0.0};    // |integral contribution| limit (same units as the output)
    double out_max{1.0};  // |output| limit, output is a normalized effort in [-out_max, out_max]
};

struct PidTerms
{
    double p{0.0};
    double i{0.0};
    double d{0.0};
    double out{0.0};
    bool saturated{false};
};

/// Single-axis PID with:
///  * derivative supplied externally (use the measured velocity -> no derivative kick
///    when the target waypoint changes, no amplification of position noise),
///  * integral clamping + conditional integration anti-windup,
///  * output saturation,
///  * dt sanity guards.
class AxisPid
{
public:
    AxisPid() = default;
    explicit AxisPid(const PidGains & g) : gains_(g) {}

    void setGains(const PidGains & g)
    {
        gains_ = g;
        clampIntegral();
    }
    const PidGains & gains() const { return gains_; }

    void reset()
    {
        integral_ = 0.0;
        terms_ = PidTerms{};
    }

    /// @param error       setpoint - measurement
    /// @param error_rate  d(error)/dt. For a (piecewise) constant setpoint this is
    ///                    simply -measured_velocity.
    /// @param dt          seconds since the last update. If dt is outside
    ///                    [min_dt, max_dt] the integrator is frozen for this step.
    double update(double error, double error_rate, double dt)
    {
        const bool dt_ok = std::isfinite(dt) && dt > min_dt_ && dt < max_dt_;

        const double p = gains_.kp * error;
        const double d = gains_.kd * error_rate;

        // Tentative output with the *previous* integral -> used to decide whether
        // integrating would push further into saturation (conditional integration).
        const double unsat = p + gains_.ki * integral_ + d;
        const bool pushing_out = (unsat > gains_.out_max && error > 0.0) ||
                                 (unsat < -gains_.out_max && error < 0.0);

        if (dt_ok && gains_.ki > 0.0 && !pushing_out) {
            integral_ += error * dt;
            clampIntegral();
        }

        const double i = gains_.ki * integral_;
        double out = p + i + d;
        const bool saturated = std::abs(out) > gains_.out_max;
        out = std::clamp(out, -gains_.out_max, gains_.out_max);

        terms_ = PidTerms{p, i, d, out, saturated};
        return out;
    }

    const PidTerms & terms() const { return terms_; }
    double integral() const { return integral_; }

    void setDtLimits(double min_dt, double max_dt)
    {
        min_dt_ = min_dt;
        max_dt_ = max_dt;
    }

private:
    void clampIntegral()
    {
        if (gains_.ki <= 0.0) {
            integral_ = 0.0;
            return;
        }
        const double lim = gains_.i_max / gains_.ki;  // i_max is expressed on the output
        integral_ = std::clamp(integral_, -lim, lim);
    }

    PidGains gains_{};
    PidTerms terms_{};
    double integral_{0.0};
    double min_dt_{1e-4};
    double max_dt_{0.5};
};

/// Wrap an angle to (-pi, pi].
inline double wrapPi(double a)
{
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0.0) a += 2.0 * M_PI;
    return a - M_PI;
}

/// Rotate a world (ENU) planar vector into the body (yaw-only) frame.
inline void worldToBody(double yaw, double ex, double ey, double & e_fwd, double & e_lat)
{
    const double c = std::cos(yaw), s = std::sin(yaw);
    e_fwd = c * ex + s * ey;
    e_lat = -s * ex + c * ey;
}

/// Map a normalized effort in [-1, 1] to an RC PWM value, compensating the
/// autopilot's RC dead-zone so that even tiny efforts produce some thrust
/// (ArduSub ignores |pwm - 1500| < RCx_DZ; in ALT_HOLD the throttle dead-zone
/// THR_DZ is 100 by default).
///
/// @param effort        normalized command, positive = "forward/left/up/ccw" in ROS body frame
/// @param reverse       flip sign (ROS ENU/FLU vs. ArduSub stick convention)
/// @param deadzone      autopilot dead-zone in PWM units for this channel
/// @param max_delta     maximum |pwm - 1500| (<= 400)
/// @param effort_eps    |effort| below this returns exactly 1500 (lets ArduSub hold depth/heading)
inline uint16_t effortToPwm(double effort, bool reverse, int deadzone, int max_delta, double effort_eps)
{
    if (!std::isfinite(effort)) return 1500;
    if (reverse) effort = -effort;
    effort = std::clamp(effort, -1.0, 1.0);
    if (std::abs(effort) < effort_eps) return 1500;

    max_delta = std::clamp(max_delta, 0, 400);
    deadzone = std::clamp(deadzone, 0, max_delta);
    const double span = static_cast<double>(max_delta - deadzone);
    // +1 so we land strictly outside the autopilot dead-zone
    const double delta = deadzone + 1 + std::abs(effort) * (span - 1);
    const int pwm = static_cast<int>(std::lround(1500.0 + (effort > 0 ? delta : -delta)));
    return static_cast<uint16_t>(std::clamp(pwm, 1100, 1900));
}

}  // namespace nav_ctrl

#endif  // AUTONOMOUS_NAVIGATION_PID_HPP
