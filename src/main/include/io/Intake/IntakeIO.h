#pragma once

#include <units/current.h>
#include <units/voltage.h>

struct IntakeIOInputs {
    double position{0.0};   // pivot position, motor rotations
    double setpoint{0.0};   // pivot setpoint, motor rotations
    double rollerRPM{0.0};  // intake roller speed
    units::volt_t pivotPIDOut{0_V};
    units::volt_t pivotFFOut{0_V};
    units::volt_t intakeVolts{0_V};
    units::ampere_t intakeCurrent{0_A};

    bool pivotAtSetpoint{false};
};

class IntakeIO {
    public:
        virtual void UpdateInputs(IntakeIOInputs& inputs) = 0;
        virtual void SetIntakeVolts(units::volt_t volts) = 0;
        virtual void SetAgitateVolts(units::volt_t volts) = 0;
        virtual void SetPivotVolts(units::volt_t volts) = 0;
        virtual void SetIntakeSetpoint(double rot) = 0;
        virtual void Agitate(double rot) = 0;
        virtual ~IntakeIO() = default;
};
