#pragma once

#include "../ForwardDeclarations.h"
#include "Math.h"
#include "Plane.h"
#include "Sphere.h"
#include "AxisAlignedBox.h"

//#include "utils.h"

namespace SoftBodyLib {
    namespace Util {

        /** Representation of a ray in space, i.e. a line with an origin and direction. */
        class Ray {
        private:
            glm::vec3 mOrigin;
            glm::vec3 mDirection;

        public:
            Ray() :mOrigin(glm::vec3(0, 0, 0)), mDirection(glm::vec3(0, 0, 1)) {}
            Ray(const glm::vec3& origin, const glm::vec3& direction)
                :mOrigin(origin), mDirection(direction) {}

            /** Sets the origin of the ray. */
            void setOrigin(const glm::vec3& origin) { mOrigin = origin; }
            /** Gets the origin of the ray. */
            const glm::vec3& getOrigin(void) const { return mOrigin; }

            /** Sets the direction of the ray. */
            void setDirection(const glm::vec3& dir) { mDirection = dir; }
            /** Gets the direction of the ray. */
            const glm::vec3& getDirection(void) const { return mDirection; }

            /** Gets the position of a point t units along the ray. */
            glm::vec3 getPoint(Real t) const {
                return glm::vec3(mOrigin + (mDirection * t));
            }

            /** Gets the position of a point t units along the ray. */
            glm::vec3 operator*(Real t) const {
                return getPoint(t);
            }

            /** Tests whether this ray intersects the given plane. */
            RayTestResult intersects(const Plane& p) const
            {
                Real denom = glm::dot(p.normal, mDirection);//p.normal.dotProduct(mDirection);
                if (Math::Abs(denom) < std::numeric_limits<Real>::epsilon())
                {
                    // Parallel
                    return RayTestResult(false, (Real)0);
                }
                else
                {
                    Real nom = glm::dot(p.normal, mOrigin) + p.d;//p.normal.dotProduct(mOrigin) + p.d;
                    Real t = -(nom / denom);
                    return RayTestResult(t >= 0, (Real)t);
                }
            }

        };

    }
}