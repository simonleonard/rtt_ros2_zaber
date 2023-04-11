#include <zaber/motion/units.h>

constexpr double kTXHome = 12.5;
constexpr double kTxLowerLimit = 7.5;
constexpr double kTxUpperLimit = 17.5;

constexpr double kTzHome = 10.0;
constexpr double kTzLowerLimit = 5.0;
constexpr double kTZUpperLimit = 15.0;

constexpr double kLsHome = 20.0;
constexpr double kLsLowerLimit = 20.0;
constexpr double kLsUpperLimit = 120.0;

constexpr double kDefaultVel = 2.5; /* mm / s */
constexpr double kDefaultAccel = 0.5; /* mm / s^2 */

constexpr double kControlTargetThreshold = 0.1; /* mm */
constexpr double kMaxControlVel = 1.0;

constexpr zaber::motion::Units kLenUnitMM =
    zaber::motion::Units::LENGTH_MILLIMETRES;
constexpr zaber::motion::Units kVelUnitMMPS =
    zaber::motion::Units::VELOCITY_MILLIMETRES_PER_SECOND;
constexpr zaber::motion::Units kAccelUnitMMPS2 =
    zaber::motion::Units::ACCELERATION_MILLIMETRES_PER_SECOND_SQUARED;
