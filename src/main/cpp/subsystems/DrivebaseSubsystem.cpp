#include "subsystems/DrivebaseSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include "Constants.h"


using namespace DrivebaseConstants;

DrivebaseSubsystem::DrivebaseSubsystem():
    frontLeft{},
    frontRight{},
    backLeft{},
    backRight{} {
        realRotationController.SetTolerance(0.5_deg, 30_deg_per_s);
        realRotationController.EnableContinuousInput(-180_deg, 180_deg);
    }