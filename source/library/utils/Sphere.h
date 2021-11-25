#pragma once

#include "../ForwardDeclarations.h"
#include "Math.h"
#include "Plane.h"

namespace SoftBodyLib {
    namespace Util {

        class Sphere
        {
        private:
            Real mRadius;
            glm::vec3 mCenter;
        public:
            /** Standard constructor - creates a unit sphere around the origin.*/
            Sphere() : mRadius(1.0), mCenter(glm::vec3(0, 0, 0)) {}
            /** Constructor allowing arbitrary spheres.
                @param center The center point of the sphere.
                @param radius The radius of the sphere.
            */
            Sphere(const glm::vec3& center, Real radius)
                : mRadius(radius), mCenter(center) {}

            /** Returns the radius of the sphere. */
            Real getRadius(void) const { return mRadius; }

            /** Sets the radius of the sphere. */
            void setRadius(Real radius) { mRadius = radius; }

            /** Returns the center point of the sphere. */
            const glm::vec3& getCenter(void) const { return mCenter; }

            /** Sets the center point of the sphere. */
            void setCenter(const glm::vec3& center) { mCenter = center; }

            /** Returns whether or not this sphere intersects another sphere. */
            bool intersects(const Sphere& s) const
            {
                return glm::length2(s.mCenter - mCenter) <=
                    Math::Sqr(s.mRadius + mRadius);
            }
            /** Returns whether or not this sphere intersects a box. */
            bool intersects(const AxisAlignedBox& box) const
            {
                return Math::intersects(*this, box);
            }
            /** Returns whether or not this sphere intersects a plane. */
            bool intersects(const Plane& plane) const
            {
                return Math::Abs(plane.getDistance(getCenter())) <= getRadius();
            }
            /** Returns whether or not this sphere intersects a point. */
            bool intersects(const glm::vec3& v) const
            {
                return (glm::length2(v - mCenter) <= Math::Sqr(mRadius));
            }
            /** Merges another Sphere into the current sphere */
            void merge(const Sphere& oth)
            {
                glm::vec3 diff = oth.getCenter() - mCenter;
                Real lengthSq = glm::length2(diff);
                Real radiusDiff = oth.getRadius() - mRadius;

                // Early-out
                if (Math::Sqr(radiusDiff) >= lengthSq)
                {
                    // One fully contains the other
                    if (radiusDiff <= 0.0f)
                        return; // no change
                    else
                    {
                        mCenter = oth.getCenter();
                        mRadius = oth.getRadius();
                        return;
                    }
                }

                Real length = Math::Sqrt(lengthSq);
                Real t = (length + radiusDiff) / (2.0f * length);
                mCenter = mCenter + diff * t;
                mRadius = 0.5f * (length + mRadius + oth.getRadius());
            }
        };

        inline bool Math::intersects(const Sphere& sphere, const Plane& plane)
        {
            return sphere.intersects(plane);
        }
    }
}