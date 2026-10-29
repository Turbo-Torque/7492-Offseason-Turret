#pragma once

#include <array>
#include <functional>

#include "frc2/command/CommandPtr.h"
#include "frc2/command/SubsystemBase.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "ctre/phoenix6/CANBus.hpp"
#include "turbolib/motors/NeoKrakenModule.hpp"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"


class DrivebaseSubsystem final:public frc2::SubsystemBase {
    public:
    DrivebaseSubsystem();

    void Drive();
    void SetModuleStates(const std::array<frc::SwerveModuleState, 4>& states);

    frc::Rotation2d GetHeading();
    frc::ChassisSpeeds GetChassisSpeeds();
    std::array<frc::SwerveModulePosition, 4> GetModulePositions();
    std::array<frc::SwerveModuleState, 4> GetModuleStates();

    frc2::CommandPtr DriveCommand(std::function<double()> xSpeed, std::function<double()> ySpeed, std::function<double()> rot);

    void Periodic() override;
    private:
        ctre::phoenix6::CANBus canBus{};
        turbolib::motors::NeoKrakenModule frontLeft, frontRight, backLeft, backRight;

        frc::PIDController rotationController{5, 0,0};
        frc::ChassisSpeeds chassisSpeeds{0_mps, 0_mps, 0_rad_per_s};
};