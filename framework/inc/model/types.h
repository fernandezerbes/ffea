#ifndef FFEA_FRAMEWORK_INC_MODEL_TYPES_H_
#define FFEA_FRAMEWORK_INC_MODEL_TYPES_H_

#include <functional>
#include "../geometry/coordinates.h"

namespace ffea {

using ConditionFunction =
    std::function<std::vector<double>(const Coordinates &)>;

} // namespace ffea

#endif // FFEA_FRAMEWORK_INC_MODEL_TYPES_H_