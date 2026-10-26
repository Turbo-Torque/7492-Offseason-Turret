#pragma once

#include "io/Intake/IntakeIO.h"
#include "Constants.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <frc/controller/ArmFeedforward.h>
#include <rev/ClosedLoopTypes.h>
#include <rev/ConfigureTypes.h>
#include <rev/SparkLowLevel.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkBaseConfig.h>
#include <rev/config/SparkMaxConfig.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/voltage.h>

class IntakeRealIO : public IntakeIO {
    public:
        IntakeRealIO() {
            ConfigIntakeMotor(intake1Motor);
            ConfigIntakeMotor(intake2Motor);
            ConfigPivotMotor();
        }

        void UpdateInputs(IntakeIOInputs& inputs) override {
            inputs.position = pivotMotor.GetEncoder().GetPosition();
            inputs.setpoint = pivotMotor.GetClosedLoopController().GetSetpoint();
            inputs.pivotAtSetpoint = pivotMotor.GetClosedLoopController().IsAtSetpoint();
            inputs.pivotFFOut = lastPivotFF;
            inputs.pivotPIDOut = units::volt_t{pivotMotor.GetAppliedOutput() * pivotMotor.GetBusVoltage()} - lastPivotFF;

            // Roller telemetry comes from the leader motor.
            inputs.intakeVolts = intake1Motor.GetMotorVoltage().GetValue();
            inputs.intakeCurrent = intake1Motor.GetTorqueCurrent().GetValue();
            inputs.rollerRPM = intake1Motor.GetVelocity().GetValue().value() * 60.0;
        }

        void SetIntakeVolts(units::volt_t voltage) override {
            // The second roller motor is mounted opposite the first, so it gets the negated voltage.
            intake1Motor.SetControl(voltageRequest.WithOutput(voltage));
            intake2Motor.SetControl(voltageRequest.WithOutput(-voltage));
        }

        void SetAgitateVolts(units::volt_t voltage) override {
            SetIntakeVolts(voltage);
        }

        void SetPivotVolts(units::volt_t voltage) override {
            lastPivotFF = 0_V;
            pivotMotor.SetVoltage(voltage);
        }

        void SetIntakeSetpoint(double rot) override {
            SetPivotPosition(rot, rev::spark::kSlot0);
        }

        // Agitate uses the softer slot 1 gains.
        void Agitate(double rot) override {
            SetPivotPosition(rot, rev::spark::kSlot1);
        }

    private:
        ctre::phoenix6::hardware::TalonFX intake1Motor{IntakeConstants::kintake1MotorPort};
        ctre::phoenix6::hardware::TalonFX intake2Motor{IntakeConstants::kintake2MotorPort};
        rev::spark::SparkMax pivotMotor{IntakeConstants::kIntakePivotPort, rev::spark::SparkLowLevel::MotorType::kBrushless};
        ctre::phoenix6::controls::VoltageOut voltageRequest{0_V};
        units::volt_t lastPivotFF{0_V};

        frc::ArmFeedforward pivotFF{
            units::volt_t{IntakeConstants::kSPivot},
            units::volt_t{IntakeConstants::kGPivot},
            units::unit_t<frc::ArmFeedforward::kv_unit>{IntakeConstants::kVPivot}};

        void SetPivotPosition(double rot, rev::spark::ClosedLoopSlot slot) {
            // TODO: the encoder reads motor rotations, but ArmFeedforward expects the arm
            // angle measured from horizontal. Add the gear ratio and zero offset here.
            units::radian_t currentAngle{pivotMotor.GetEncoder().GetPosition()};
            lastPivotFF = pivotFF.Calculate(currentAngle, 0_rad_per_s);
            pivotMotor.GetClosedLoopController().SetSetpoint(
                rot, rev::spark::SparkLowLevel::ControlType::kPosition, slot, lastPivotFF.value());
        }

        static void ConfigIntakeMotor(ctre::phoenix6::hardware::TalonFX& motor) {
            ctre::phoenix6::configs::TalonFXConfiguration config{};
            config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
            config.CurrentLimits.SupplyCurrentLimit = 45_A;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;

            motor.GetConfigurator().Apply(config);
        }

        void ConfigPivotMotor() {
            rev::spark::SparkMaxConfig config;
            config.closedLoop.P(0.04, rev::spark::kSlot0);
            config.closedLoop.I(0.001, rev::spark::kSlot0);
            config.closedLoop.D(0.002, rev::spark::kSlot0);

            config.closedLoop.P(0.02, rev::spark::kSlot1);
            config.closedLoop.I(0.0, rev::spark::kSlot1);
            config.closedLoop.D(0.002, rev::spark::kSlot1);

            config.SmartCurrentLimit(35, 50);
            config.SetIdleMode(rev::spark::SparkBaseConfig::kCoast);
            config.OpenLoopRampRate(0.1);
            config.closedLoop.AllowedClosedLoopError(0.20);
            config.Inverted(true);

            pivotMotor.Configure(config, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
            pivotMotor.GetEncoder().SetPosition(0.0);
        }
};
