#pragma once

#include <units/voltage.h>

// TODO: every value below is a placeholder. Set the real CAN IDs, setpoints,
// and feedforward gains for this robot before running on hardware.
namespace IntakeConstants {
    // CAN IDs
    inline constexpr int kintake1MotorPort = 1;  // TalonFX roller (leader)
    inline constexpr int kintake2MotorPort = 2;  // TalonFX roller (spins opposite)
    inline constexpr int kIntakePivotPort = 3;   // SparkMax pivot

    // Pivot setpoints, in pivot motor rotations
    inline constexpr double kIntakeDown = 0.0;
    inline constexpr double kIntakeHalfway = 0.0;
    inline constexpr double kIntakeAgitate = 0.0;

    // Voltages
    inline constexpr units::volt_t kIntakeVolts = 0_V;
    inline constexpr units::volt_t kOuttakeVolts = 0_V;
    inline constexpr units::volt_t kIntakeAgitateVolts = 0_V;
    inline constexpr units::volt_t kPivotVolts = 0_V;

    // Pivot arm feedforward gains
    inline constexpr double kSPivot = 0.0;  // volts
    inline constexpr double kGPivot = 0.0;  // volts
    inline constexpr double kVPivot = 0.0;  // volts per (rad/s)
}  // namespace IntakeConstants
