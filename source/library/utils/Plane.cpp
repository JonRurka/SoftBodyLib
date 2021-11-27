#include "utils.h"

namespace SoftBodyLib {
    namespace Util {

        Plane::Side Plane::getSide(const AxisAlignedBox& box) const
        {
            return NO_SIDE;

            if (box.isNull())
                return NO_SIDE;
            if (box.isInfinite())
                return BOTH_SIDE;

            return getSide(box.getCenter(), box.getHalfSize());
        }
    }
}