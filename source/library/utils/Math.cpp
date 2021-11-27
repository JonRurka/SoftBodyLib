#include "utils.h"

namespace SoftBodyLib {
    namespace Util {

        #define INFINITE_FAR_PLANE_ADJUST  0.00001

        int Math::mTrigTableSize;
        Math::AngleUnit Math::msAngleUnit;

        float  Math::mTrigTableFactor;
        float* Math::mSinTable = NULL;
        float* Math::mTanTable = NULL;

        //Math::RandomValueProvider* Math::mRandProvider = NULL;

        


        Math::Math(unsigned int trigTableSize)
        {
            msAngleUnit = AU_DEGREE;
            mTrigTableSize = trigTableSize;
            mTrigTableFactor = mTrigTableSize / Math::TWO_PI;

            //mSinTable = OGRE_ALLOC_T(float, mTrigTableSize, MEMCATEGORY_GENERAL);
            //mTanTable = OGRE_ALLOC_T(float, mTrigTableSize, MEMCATEGORY_GENERAL);

            buildTrigTables();
        }

        //-----------------------------------------------------------------------
        Math::~Math()
        {
            //OGRE_FREE(mSinTable, MEMCATEGORY_GENERAL);
            //OGRE_FREE(mTanTable, MEMCATEGORY_GENERAL);
        }


        void Math::buildTrigTables(void)
        {
            // Build trig lookup tables
            // Could get away with building only PI sized Sin table but simpler this 
            // way. Who cares, it'll ony use an extra 8k of memory anyway and I like 
            // simplicity.
            float angle;
            for (int i = 0; i < mTrigTableSize; ++i)
            {
                angle = Math::TWO_PI * i / Real(mTrigTableSize);
                mSinTable[i] = std::sin(angle);
                mTanTable[i] = std::tan(angle);
            }
        }

        float Math::SinTable(float fValue)
        {
            // Convert range to index values, wrap if required
            int idx;
            if (fValue >= 0)
            {
                idx = int(fValue * mTrigTableFactor) % mTrigTableSize;
            }
            else
            {
                idx = mTrigTableSize - (int(-fValue * mTrigTableFactor) % mTrigTableSize) - 1;
            }

            return mSinTable[idx];
        }

        //-----------------------------------------------------------------------
        float Math::TanTable(float fValue)
        {
            // Convert range to index values, wrap if required
            int idx = int(fValue *= mTrigTableFactor) % mTrigTableSize;
            return mTanTable[idx];
        }
        
        //-----------------------------------------------------------------------
        Radian Math::ACos(Real fValue)
        {
            if (-1.0 < fValue)
            {
                if (fValue < 1.0)
                    return Radian(std::acos(fValue));
                else
                    return Radian(0.0);
            }
            else
            {
                return Radian(PI);
            }
        }
        
        //-----------------------------------------------------------------------
        Radian Math::ASin(Real fValue)
        {
            if (-1.0 < fValue)
            {
                if (fValue < 1.0)
                    return Radian(std::asin(fValue));
                else
                    return Radian(HALF_PI);
            }
            else
            {
                return Radian(-HALF_PI);
            }
        }
        
        //-----------------------------------------------------------------------
        Real Math::UnitRandom()
        {
            //TODO:
            /*if (mRandProvider)
                return mRandProvider->getRandomUnit();
            else return Real(rand()) / float(RAND_MAX);*/

            return Real(rand()) / float(RAND_MAX);
        }

        //-----------------------------------------------------------------------
        /*void Math::SetRandomValueProvider(RandomValueProvider* provider)
        {
            mRandProvider = provider;
        }*/

        //-----------------------------------------------------------------------
        void Math::setAngleUnit(Math::AngleUnit unit)
        {
            msAngleUnit = unit;
        }
        
        //-----------------------------------------------------------------------
        Math::AngleUnit Math::getAngleUnit(void)
        {
            return msAngleUnit;
        }
        
        //-----------------------------------------------------------------------
        float Math::AngleUnitsToRadians(float angleunits)
        {
            if (msAngleUnit == AU_DEGREE)
                return angleunits * fDeg2Rad;
            else
                return angleunits;
        }

        //-----------------------------------------------------------------------
        float Math::RadiansToAngleUnits(float radians)
        {
            if (msAngleUnit == AU_DEGREE)
                return radians * fRad2Deg;
            else
                return radians;
        }

        //-----------------------------------------------------------------------
        float Math::AngleUnitsToDegrees(float angleunits)
        {
            if (msAngleUnit == AU_RADIAN)
                return angleunits * fRad2Deg;
            else
                return angleunits;
        }

        //-----------------------------------------------------------------------
        float Math::DegreesToAngleUnits(float degrees)
        {
            if (msAngleUnit == AU_RADIAN)
                return degrees * fDeg2Rad;
            else
                return degrees;
        }

        //-----------------------------------------------------------------------
        bool Math::pointInTri2D(const glm::vec2& p, const glm::vec2& a,
            const glm::vec2& b, const glm::vec2& c)
        {
            // Winding must be consistent from all edges for point to be inside
            glm::vec2 v1, v2;
            Real dot[3];
            bool zeroDot[3];

            v1 = b - a;
            v2 = p - a;

            // Note we don't care about normalisation here since sign is all we need
            // It means we don't have to worry about magnitude of cross products either
            //dot[0] = v1.crossProduct(v2);
            dot[0] = crossProduct(v1, v2);
            zeroDot[0] = Math::RealEqual(dot[0], 0.0f, 1e-3);


            v1 = c - b;
            v2 = p - b;


            //dot[1] = v1.crossProduct(v2);
            dot[1] = crossProduct(v1, v2);
            zeroDot[1] = Math::RealEqual(dot[1], 0.0f, 1e-3);

            // Compare signs (ignore colinear / coincident points)
            if (!zeroDot[0] && !zeroDot[1]
                && Math::Sign(dot[0]) != Math::Sign(dot[1]))
            {
                return false;
            }

            v1 = a - c;
            v2 = p - c;

            dot[2] = crossProduct(v1, v2);
            zeroDot[2] = Math::RealEqual(dot[2], 0.0f, 1e-3);
            // Compare signs (ignore colinear / coincident points)
            if ((!zeroDot[0] && !zeroDot[2]
                && Math::Sign(dot[0]) != Math::Sign(dot[2])) ||
                (!zeroDot[1] && !zeroDot[2]
                    && Math::Sign(dot[1]) != Math::Sign(dot[2])))
            {
                return false;
            }


            return true;
        }
        
        //-----------------------------------------------------------------------
        bool Math::pointInTri3D(const glm::vec3& p, const glm::vec3& a,
            const glm::vec3& b, const glm::vec3& c, const glm::vec3& normal)
        {
            // Winding must be consistent from all edges for point to be inside
            glm::vec3 v1, v2;
            Real dot[3];
            bool zeroDot[3];

            v1 = b - a;
            v2 = p - a;

            // Note we don't care about normalisation here since sign is all we need
            // It means we don't have to worry about magnitude of cross products either
            //dot[0] = v1.crossProduct(v2).dotProduct(normal);
            dot[0] = glm::dot(glm::cross(v1, v2), normal);
            zeroDot[0] = Math::RealEqual(dot[0], 0.0f, 1e-3);


            v1 = c - b;
            v2 = p - b;

            //dot[1] = v1.crossProduct(v2).dotProduct(normal);
            dot[1] = glm::dot(glm::cross(v1, v2), normal);
            zeroDot[1] = Math::RealEqual(dot[1], 0.0f, 1e-3);

            // Compare signs (ignore colinear / coincident points)
            if (!zeroDot[0] && !zeroDot[1]
                && Math::Sign(dot[0]) != Math::Sign(dot[1]))
            {
                return false;
            }

            v1 = a - c;
            v2 = p - c;

            //dot[2] = v1.crossProduct(v2).dotProduct(normal);
            dot[2] = glm::dot(glm::cross(v1, v2), normal);
            zeroDot[2] = Math::RealEqual(dot[2], 0.0f, 1e-3);
            // Compare signs (ignore colinear / coincident points)
            if ((!zeroDot[0] && !zeroDot[2]
                && Math::Sign(dot[0]) != Math::Sign(dot[2])) ||
                (!zeroDot[1] && !zeroDot[2]
                    && Math::Sign(dot[1]) != Math::Sign(dot[2])))
            {
                return false;
            }


            return true;
        }
        
        // used?
        //-----------------------------------------------------------------------
        std::pair<bool, Real> Math::intersects(const Ray& ray, const glm::vec3& a, const glm::vec3& b,
            const glm::vec3& c, bool positiveSide, bool negativeSide)
        {
            const Real EPSILON = 1e-6f;
            glm::vec3 E1 = b - a;
            glm::vec3 E2 = c - a;
            glm::vec3 P = glm::cross(ray.getDirection(), E2);
            Real det = glm::dot(E1, P);
            //Real det = E1.dotProduct(P);

            // if determinant is near zero, ray lies in plane of triangle
            if ((!positiveSide || det <= EPSILON) && (!negativeSide || det >= -EPSILON))
                return { false, (Real)0 };
            Real inv_det = 1.0f / det;

            // calculate u parameter and test bounds
            glm::vec3 T = ray.getOrigin() - a;
            Real u = glm::dot(T, P) *inv_det;
            if (u < 0.0f || u > 1.0f)
                return { false, (Real)0 };

            // calculate v parameter and test bounds
            //glm::vec3 Q = T.crossProduct(E1);
            glm::vec3 Q = glm::cross(T, E1);
            Real v = glm::dot(ray.getDirection(), Q) * inv_det; //ray.getDirection().dotProduct(Q) * inv_det;
            if (v < 0.0f || u + v > 1.0f)
                return { false, (Real)0 };

            // calculate t, ray intersects triangle
            Real t = glm::dot(E2, Q) * inv_det;
            if (t < 0.0f)
                return { false, (Real)0 };

            return { true, t };
        }
        
        //-----------------------------------------------------------------------
        bool Math::intersects(const Sphere& sphere, const AxisAlignedBox& box)
        {
            if (box.isNull()) return false;
            if (box.isInfinite()) return true;

            // Use splitting planes
            const glm::vec3& center = sphere.getCenter();
            Real radius = sphere.getRadius();
            const glm::vec3& min = box.getMinimum();
            const glm::vec3& max = box.getMaximum();

            // Arvo's algorithm
            Real s, d = 0;
            for (int i = 0; i < 3; ++i)
            {
                if (center[i] < min[i])
                {
                    s = center[i] - min[i];
                    d += s * s;
                }
                else if (center[i] > max[i])
                {
                    s = center[i] - max[i];
                    d += s * s;
                }
            }
            return d <= radius * radius;

        }
        
        //-----------------------------------------------------------------------
        std::pair<bool, Real> Math::intersects(const Ray& ray, const AxisAlignedBox& box)
        {
            if (box.isNull()) return std::pair<bool, Real>(false, (Real)0);
            if (box.isInfinite()) return std::pair<bool, Real>(true, (Real)0);

            Real lowt = 0.0f;
            Real t;
            bool hit = false;
            glm::vec3 hitpoint;
            const glm::vec3& min = box.getMinimum();
            const glm::vec3& max = box.getMaximum();
            const glm::vec3& rayorig = ray.getOrigin();
            const glm::vec3& raydir = ray.getDirection();

            // Check origin inside first
            //glm::greaterThan()
            if (glm::all(glm::greaterThan(rayorig, min)) && glm::all(glm::lessThan(rayorig, max)))
            {
                return std::pair<bool, Real>(true, (Real)0);
            }

            // Check each face in turn, only check closest 3
            // Min x
            if (rayorig.x <= min.x && raydir.x > 0)
            {
                t = (min.x - rayorig.x) / raydir.x;

                // Substitute t back into ray and check bounds and dist
                hitpoint = rayorig + raydir * t;
                if (hitpoint.y >= min.y && hitpoint.y <= max.y &&
                    hitpoint.z >= min.z && hitpoint.z <= max.z &&
                    (!hit || t < lowt))
                {
                    hit = true;
                    lowt = t;
                }
            }
            // Max x
            if (rayorig.x >= max.x && raydir.x < 0)
            {
                t = (max.x - rayorig.x) / raydir.x;

                // Substitute t back into ray and check bounds and dist
                hitpoint = rayorig + raydir * t;
                if (hitpoint.y >= min.y && hitpoint.y <= max.y &&
                    hitpoint.z >= min.z && hitpoint.z <= max.z &&
                    (!hit || t < lowt))
                {
                    hit = true;
                    lowt = t;
                }
            }
            // Min y
            if (rayorig.y <= min.y && raydir.y > 0)
            {
                t = (min.y - rayorig.y) / raydir.y;

                // Substitute t back into ray and check bounds and dist
                hitpoint = rayorig + raydir * t;
                if (hitpoint.x >= min.x && hitpoint.x <= max.x &&
                    hitpoint.z >= min.z && hitpoint.z <= max.z &&
                    (!hit || t < lowt))
                {
                    hit = true;
                    lowt = t;
                }
            }
            // Max y
            if (rayorig.y >= max.y && raydir.y < 0)
            {
                t = (max.y - rayorig.y) / raydir.y;

                // Substitute t back into ray and check bounds and dist
                hitpoint = rayorig + raydir * t;
                if (hitpoint.x >= min.x && hitpoint.x <= max.x &&
                    hitpoint.z >= min.z && hitpoint.z <= max.z &&
                    (!hit || t < lowt))
                {
                    hit = true;
                    lowt = t;
                }
            }
            // Min z
            if (rayorig.z <= min.z && raydir.z > 0)
            {
                t = (min.z - rayorig.z) / raydir.z;

                // Substitute t back into ray and check bounds and dist
                hitpoint = rayorig + raydir * t;
                if (hitpoint.x >= min.x && hitpoint.x <= max.x &&
                    hitpoint.y >= min.y && hitpoint.y <= max.y &&
                    (!hit || t < lowt))
                {
                    hit = true;
                    lowt = t;
                }
            }
            // Max z
            if (rayorig.z >= max.z && raydir.z < 0)
            {
                t = (max.z - rayorig.z) / raydir.z;

                // Substitute t back into ray and check bounds and dist
                hitpoint = rayorig + raydir * t;
                if (hitpoint.x >= min.x && hitpoint.x <= max.x &&
                    hitpoint.y >= min.y && hitpoint.y <= max.y &&
                    (!hit || t < lowt))
                {
                    hit = true;
                    lowt = t;
                }
            }

            return std::pair<bool, Real>(hit, (Real)lowt);

        }

        //-----------------------------------------------------------------------
        glm::vec3 Math::calculateTangentSpaceVector(
            const glm::vec3& position1, const glm::vec3& position2, const glm::vec3& position3,
            Real u1, Real v1, Real u2, Real v2, Real u3, Real v3)
        {
            //side0 is the glm::vec along one side of the triangle of vertices passed in, 
            //and side1 is the glm::vec along another side. Taking the cross product of these returns the normal.
            glm::vec3 side0 = position1 - position2;
            glm::vec3 side1 = position3 - position1;
            //Calculate face normal
            glm::vec3 normal = glm::cross(side1, side0); //side1.crossProduct(side0);
            normal = glm::normalize(normal);//normal.normalise();
            //Now we use a formula to calculate the tangent. 
            Real deltaV0 = v1 - v2;
            Real deltaV1 = v3 - v1;
            glm::vec3 tangent = deltaV1 * side0 - deltaV0 * side1;
            tangent = glm::normalize(tangent);  //tangent.normalise();
            //Calculate binormal
            Real deltaU0 = u1 - u2;
            Real deltaU1 = u3 - u1;
            glm::vec3 binormal = deltaU1 * side0 - deltaU0 * side1;
            binormal = glm::normalize(binormal); //binormal.normalise();
            //Now, we take the cross product of the tangents to get a glm::vec which 
            //should point in the same direction as our normal calculated above. 
            //If it points in the opposite direction (the dot product between the normals is less than zero), 
            //then we need to reverse the s and t tangents. 
            //This is because the triangle has been mirrored when going from tangent space to object space.
            //reverse tangents if necessary
            glm::vec3 tangentCross = glm::cross(tangent, binormal); //tangent.crossProduct(binormal);
            if (glm::dot(tangentCross, normal) < 0.0f)//(tangentCross.dotProduct(normal) < 0.0f)
            {
                tangent = -tangent;
                binormal = -binormal;
            }

            return tangent;

        }
        
        //-----------------------------------------------------------------------
        /*Affine3 Math::buildReflectionMatrix(const Plane & p)
        {
            return Affine3(
                -2 * p.normal.x * p.normal.x + 1, -2 * p.normal.x * p.normal.y, -2 * p.normal.x * p.normal.z, -2 * p.normal.x * p.d,
                -2 * p.normal.y * p.normal.x, -2 * p.normal.y * p.normal.y + 1, -2 * p.normal.y * p.normal.z, -2 * p.normal.y * p.d,
                -2 * p.normal.z * p.normal.x, -2 * p.normal.z * p.normal.y, -2 * p.normal.z * p.normal.z + 1, -2 * p.normal.z * p.d);
        }*/
        
        //-----------------------------------------------------------------------
        Real Math::gaussianDistribution(Real x, Real offset, Real scale)
        {
            Real nom = Math::Exp(
                -Math::Sqr(x - offset) / (2 * Math::Sqr(scale)));
            Real denom = scale * Math::Sqrt(2 * Math::PI);

            return nom / denom;

        }
        
        //---------------------------------------------------------------------
        /*Affine3 Math::makeViewMatrix(const glm::vec3& position, const Quaternion& orientation,
            const Affine3* reflectMatrix)
        {
            // This is most efficiently done using 3x3 Matrices
            Matrix3 rot;
            orientation.ToRotationMatrix(rot);

            // Make the translation relative to new axes
            Matrix3 rotT = rot.Transpose();
            glm::vec3 trans = -rotT * position;

            // Make final matrix
            Affine3 viewMatrix = Affine3::IDENTITY;
            viewMatrix = rotT; // fills upper 3x3
            viewMatrix[0][3] = trans.x;
            viewMatrix[1][3] = trans.y;
            viewMatrix[2][3] = trans.z;

            // Deal with reflections
            if (reflectMatrix)
            {
                viewMatrix = viewMatrix * (*reflectMatrix);
            }

            return viewMatrix;

        } */

        /*glm::mat4x4 Math::makePerspectiveMatrix(Real left, Real right, Real bottom, Real top, Real zNear, Real zFar)
        {
            // The code below will dealing with general projection
            // parameters, similar glFrustum.
            // Doesn't optimise manually except division operator, so the
            // code more self-explaining.

            Real inv_w = 1 / (right - left);
            Real inv_h = 1 / (top - bottom);
            Real inv_d = 1 / (zFar - zNear);

            // Calc matrix elements
            Real A = 2 * zNear * inv_w;
            Real B = 2 * zNear * inv_h;
            Real C = (right + left) * inv_w;
            Real D = (top + bottom) * inv_h;
            Real q, qn;

            if (zFar == 0)
            {
                // Infinite far plane
                q = INFINITE_FAR_PLANE_ADJUST - 1;
                qn = zNear * (INFINITE_FAR_PLANE_ADJUST - 2);
            }
            else
            {
                q = -(zFar + zNear) * inv_d;
                qn = -2 * (zFar * zNear) * inv_d;
            }

            glm::mat4x4 ret = glm::mat4x4(0);
            ret[0][0] = A;
            ret[0][2] = C;
            ret[1][1] = B;
            ret[1][2] = D;
            ret[2][2] = q;
            ret[2][3] = qn;
            ret[3][2] = -1;

            return ret;
        }*/

        //---------------------------------------------------------------------
        Real Math::boundingRadiusFromAABB(const AxisAlignedBox& aabb)
        {
            const glm::vec3& max = aabb.getMaximum();
            const glm::vec3& min = aabb.getMinimum();

            glm::vec3 magnitude = max;

            magnitude = makeCeil(magnitude, -max);
            magnitude = makeCeil(magnitude, min);
            magnitude = makeCeil(magnitude, -min);

            //magnitude.makeCeil(-max);
            //magnitude.makeCeil(min);
            //magnitude.makeCeil(-min);

            return glm::length(magnitude);
        }

        /*Real Math::boundingRadiusFromAABBCentered(const AxisAlignedBox& aabb)
        {
            const glm::vec3& max = aabb.getMaximum();
            const glm::vec3& min = aabb.getMinimum();

            return glm::length((min - max) * 0.5f);
        }*/
        
        inline glm::vec3 Math::calculateBasicFaceNormal(const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3)
        {
            glm::vec3 normal = glm::cross((v2 - v1), (v3 - v1)) ;//(v2 - v1).crossProduct(v3 - v1);
            normal = glm::normalize(normal);
            return normal;
        }
        
        inline glm::vec4 Math::calculateFaceNormal(const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3)
        {
            glm::vec3 normal = calculateBasicFaceNormal(v1, v2, v3);
            // Now set up the w (distance of tri from origin
            return glm::vec4(normal.x, normal.y, normal.z, -( glm::dot(normal, v1) /*normal.dotProduct(v1)*/  ));
        }
        
        inline glm::vec3 Math::calculateBasicFaceNormalWithoutNormalize(
            const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3)
        {
            return glm::cross((v2 - v1), (v3 - v1)); //(v2 - v1).crossProduct(v3 - v1);
        }

        inline glm::vec4 Math::calculateFaceNormalWithoutNormalize(const glm::vec3& v1,
            const glm::vec3& v2,
            const glm::vec3& v3)
        {
            glm::vec3 normal = calculateBasicFaceNormalWithoutNormalize(v1, v2, v3);
            // Now set up the w (distance of tri from origin)
            return glm::vec4(normal.x, normal.y, normal.z, -(  glm::dot(normal, v1) /*normal.dotProduct(v1)*/  ));
        }

    }
}