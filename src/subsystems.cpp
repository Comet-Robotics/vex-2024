#include "subsystems.h"

std::unique_ptr<Drivebase> drivebase = nullptr;
std::unique_ptr<Catapult> catapult = nullptr;
std::unique_ptr<Intake> intake = nullptr;
std::unique_ptr<Wings> wings = nullptr;

void subsystems_initialize() {
	catapult = std::make_unique<Catapult>();
	drivebase = std::make_unique<Drivebase>();
	intake = std::make_unique<Intake>();
	wings = std::make_unique<Wings>();
}
