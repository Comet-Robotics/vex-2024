#ifndef __SUBSYSTEMS_H__
#define __SUBSYSTEMS_H__

#include <memory>
#include "subsystems/catapult.h"
#include "subsystems/drivebase.h"
#include "subsystems/intake.h"

extern std::unique_ptr<Drivebase> drivebase;
extern std::unique_ptr<Catapult> catapult;
extern std::unique_ptr<Intake> intake;

void subsystems_initialize();

#endif
