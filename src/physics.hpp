#pragma once
#include "sim.hpp"

void initScene(Scene& s);
void resetRobot(Scene& s);
void stepPhysics(Scene& s, const MotorCmd& m, float dt);
