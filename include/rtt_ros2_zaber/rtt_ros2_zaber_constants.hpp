#pragma once

#include <zaber/motion/units.h>

const int kNumDevices = 3;

constexpr double kTxHome = 11.5;
constexpr double kTxLowerLimit = 6.5;
constexpr double kTxUpperLimit = 16.5;

constexpr double kLsHome = 20.0;
constexpr double kLsLowerLimit = 20.0;
constexpr double kLsUpperLimit = 120.0;

constexpr double kTzHome = 9.0;
constexpr double kTzLowerLimit = 4.0;
constexpr double kTzUpperLimit = 14.0;

constexpr double kDefaultVel = 2.5;   /* mm / s */
constexpr double kDefaultAccel = 0.5; /* mm / s^2 */

constexpr double kControlTargetThreshold = 0.1; /* mm */

constexpr zaber::motion::Units kLenUnitMM =
    zaber::motion::Units::LENGTH_MILLIMETRES;
constexpr zaber::motion::Units kVelUnitMMPS =
    zaber::motion::Units::VELOCITY_MILLIMETRES_PER_SECOND;
constexpr zaber::motion::Units kAccelUnitMMPS2 =
    zaber::motion::Units::ACCELERATION_MILLIMETRES_PER_SECOND_SQUARED;
