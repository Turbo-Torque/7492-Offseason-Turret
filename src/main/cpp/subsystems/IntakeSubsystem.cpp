#include "subsystems/IntakeSubsystem.h"

#include <string>
#include <utility>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

#include "Constants.h"

IntakeSubsystem::IntakeSubsystem(std::unique_ptr<IntakeIO> intakeIo) : io(std::move(intakeIo)) {
    SetName("IntakeSubsystem");
}

frc2::CommandPtr IntakeSubsystem::PivotAndRunIntakeCommand() {
    return GuardedByReset(
        frc2::cmd::Run([this] { SetIntakeSetpoint(IntakeConstants::kIntakeDown + pivotOffset); }, {this})
        .Until([this] { return AtPivotSetpoint(IntakeConstants::kIntakeDown + pivotOffset); })
        .AndThen(frc2::cmd::Run([this] { SetIntakeVoltage(IntakeConstants::kIntakeVolts); }, {this}))
        .FinallyDo([this] {
            SetIntakeVoltage(0_V);
            SetIntakeSetpoint(IntakeConstants::kIntakeHalfway + pivotOffset);
        })
    );
}

frc2::CommandPtr IntakeSubsystem::PivotAndRunOuttakeCommand() {
    return GuardedByReset(
        frc2::cmd::Run([this] { SetIntakeSetpoint(IntakeConstants::kIntakeDown + pivotOffset); }, {this})
        .Until([this] { return AtPivotSetpoint(IntakeConstants::kIntakeDown + pivotOffset); })
        .AndThen(frc2::cmd::Run([this] { SetIntakeVoltage(IntakeConstants::kOuttakeVolts); }, {this}))
        .FinallyDo([this] {
            SetIntakeVoltage(0_V);
            SetIntakeSetpoint(IntakeConstants::kIntakeHalfway + pivotOffset);
        })
    );
}

frc2::CommandPtr IntakeSubsystem::AgitateCommand() {
    return GuardedByReset(
        frc2::cmd::Run([this] { SetIntakeSetpoint(IntakeConstants::kIntakeAgitate + pivotOffset); }, {this})
        .Until([this] { return AtPivotSetpoint(IntakeConstants::kIntakeAgitate + pivotOffset); })
        .AndThen(frc2::cmd::Run([this] { SetAgitateVolts(IntakeConstants::kIntakeAgitateVolts); }, {this}))
        .FinallyDo([this] {
            SetAgitateVolts(0_V);
            Agitate(IntakeConstants::kIntakeHalfway + pivotOffset);
        })
    );
}

frc2::CommandPtr IntakeSubsystem::SupplyPivotVoltsCommand() {
    return frc2::cmd::Run([this] { SetPivotVolts(IntakeConstants::kPivotVolts); }, {this})
        .FinallyDo([this] {
            SetPivotVolts(0_V);
            newIntakePivot = inputs.position;
            pivotOffset = newIntakePivot - IntakeConstants::kIntakeDown;
        });
}

frc2::CommandPtr IntakeSubsystem::PivotVolts() {
    return GuardedByReset(
        frc2::cmd::Run([this] { SetPivotVolts(IntakeConstants::kPivotVolts); }, {this})
        .FinallyDo([this] {
            SetPivotVolts(0_V);
        })
    );
}

frc2::CommandPtr IntakeSubsystem::SetIntakeToUp() {
    return GuardedByReset(
        frc2::cmd::Run([this] { SetIntakeSetpoint(0.0 + pivotOffset); }, {this})
        .FinallyDo([this] {
            SetIntakeSetpoint(0.0 + pivotOffset);
        })
    );
}

frc2::CommandPtr IntakeSubsystem::GuardedByReset(frc2::CommandPtr&& command) {
    return frc2::cmd::Either(std::move(command), frc2::cmd::None(), [this] { return !resetIntake; });
}

bool IntakeSubsystem::IntakeNeedHopper() {
    return inputs.intakeCurrent >= 50_A && inputs.rollerRPM < 3000.0;
}

bool IntakeSubsystem::AgitateRollers() {
    return true;
}

bool IntakeSubsystem::ResetIntake() {
    return resetIntake;
}

// Holds the reset flag while the command is scheduled, e.g. while a button is held.
frc2::CommandPtr IntakeSubsystem::SetResetIntake() {
    return frc2::cmd::StartEnd([this] { resetIntake = true; }, [this] { resetIntake = false; });
}

void IntakeSubsystem::Periodic() {
    io->UpdateInputs(inputs);

    std::string intakeState;
    if (inputs.position > (IntakeConstants::kIntakeDown + pivotOffset - 3) && inputs.position <= (IntakeConstants::kIntakeDown + pivotOffset + 5)) {
        intakeState = "Intake Down";
    } else if (inputs.position > (IntakeConstants::kIntakeHalfway + pivotOffset - 3) && inputs.position <= (IntakeConstants::kIntakeHalfway + pivotOffset + 5)) {
        intakeState = "Intake Halfway";
    } else if (inputs.position > (IntakeConstants::kIntakeAgitate + pivotOffset - 2) && inputs.position <= (IntakeConstants::kIntakeAgitate + pivotOffset + 5)) {
        intakeState = "Intake Agitate";
    } else {
        intakeState = "Intake Up";
    }

    frc::SmartDashboard::PutNumber("Intake Pose", inputs.position);
    frc::SmartDashboard::PutNumber("Intake Volts", inputs.intakeVolts.value());
    frc::SmartDashboard::PutBoolean("intake setpoint", inputs.pivotAtSetpoint);
    frc::SmartDashboard::PutNumber("intake current", inputs.intakeCurrent.value());
    frc::SmartDashboard::PutNumber("intake rollers rpm", inputs.rollerRPM);
    frc::SmartDashboard::PutString("Intake State", intakeState);
    frc::SmartDashboard::PutBoolean("Intake Roller Stall", IntakeNeedHopper());
}
