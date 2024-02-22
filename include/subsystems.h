#ifndef __SUBSYSTEMS_H__
#define __SUBSYSTEMS_H__

#include <memory>
#include "subsystems/catapult.h"
#include "subsystems/drivebase.h"
#include "subsystems/intake.h"
#include "subsystems/wings.h"

extern std::unique_ptr<Drivebase> drivebase;
extern std::unique_ptr<Catapult> catapult;
extern std::unique_ptr<Intake> intake;
extern std::unique_ptr<Wings> wings;

void subsystems_initialize();

#endif
