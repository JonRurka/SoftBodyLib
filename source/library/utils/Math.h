#pragma once

#include "../ForwardDeclarations.h"

namespace SoftBodyLib {
    namespace Util {

        class Radian
        {
            Real mRad;

        public:
            explicit Radian(Real r = 0) : mRad(r) {}
            Radian(const Degree& d);
            Radian& operator = (const Real& f) { mRad = f; return *this; }
            Radian& operator = (const Radian& r) { mRad = r.mRad; return *this; }
            Radian& operator = (const Degree& d);

            Real valueDegrees() const; // see bottom of this file
            Real valueRadians() const { return mRad; }
            Real valueAngleUnits() const;

            const Radian& operator + () const { return *this; }
            Radian operator + (const Radian& r) const { return Radian(mRad + r.mRad); }
            Radian operator + (const Degree& d) const;
            Radian& operator += (const Radian& r) { mRad += r.mRad; return *this; }
            Radian& operator += (const Degree& d);
            Radian operator - () const { return Radian(-mRad); }
            Radian operator - (const Radian& r) const { return Radian(mRad - r.mRad); }
            Radian operator - (const Degree& d) const;
            Radian& operator -= (const Radian& r) { mRad -= r.mRad; return *this; }
            Radian& operator -= (const Degree& d);
            Radian operator * (Real f) const { return Radian(mRad * f); }
            Radian operator * (const Radian& f) const { return Radian(mRad * f.mRad); }
            Radian& operator *= (Real f) { mRad *= f; return *this; }
            Radian operator / (Real f) const { return Radian(mRad / f); }
            Radian& operator /= (Real f) { mRad /= f; return *this; }

            bool operator <  (const Radian& r) const { return mRad < r.mRad; }
            bool operator <= (const Radian& r) const { return mRad <= r.mRad; }
            bool operator == (const Radian& r) const { return mRad == r.mRad; }
            bool operator != (const Radian& r) const { return mRad != r.mRad; }
            bool operator >= (const Radian& r) const { return mRad >= r.mRad; }
            bool operator >  (const Radian& r) const { return mRad > r.mRad; }

            inline friend std::ostream& operator <<
                (std::ostream& o, const Radian& v)
            {
                o << "Radian(" << v.valueRadians() << ")";
                return o;
            }
        };

        class Degree
        {
            Real mDeg; // if you get an error here - make sure to define/typedef 'Real' first

        public:
            explicit Degree(Real d = 0) : mDeg(d) {}
            Degree(const Radian& r) : mDeg(r.valueDegrees()) {}
            Degree& operator = (const Real& f) { mDeg = f; return *this; }
            Degree& operator = (const Degree& d) { mDeg = d.mDeg; return *this; }
            Degree& operator = (const Radian& r) { mDeg = r.valueDegrees(); return *this; }

            Real valueDegrees() const { return mDeg; }
            Real valueRadians() const; // see bottom of this file
            Real valueAngleUnits() const;

            const Degree& operator + () const { return *this; }
            Degree operator + (const Degree& d) const { return Degree(mDeg + d.mDeg); }
            Degree operator + (const Radian& r) const { return Degree(mDeg + r.valueDegrees()); }
            Degree& operator += (const Degree& d) { mDeg += d.mDeg; return *this; }
            Degree& operator += (const Radian& r) { mDeg += r.valueDegrees(); return *this; }
            Degree operator - () const { return Degree(-mDeg); }
            Degree operator - (const Degree& d) const { return Degree(mDeg - d.mDeg); }
            Degree operator - (const Radian& r) const { return Degree(mDeg - r.valueDegrees()); }
            Degree& operator -= (const Degree& d) { mDeg -= d.mDeg; return *this; }
            Degree& operator -= (const Radian& r) { mDeg -= r.valueDegrees(); return *this; }
            Degree operator * (Real f) const { return Degree(mDeg * f); }
            Degree operator * (const Degree& f) const { return Degree(mDeg * f.mDeg); }
            Degree& operator *= (Real f) { mDeg *= f; return *this; }
            Degree operator / (Real f) const { return Degree(mDeg / f); }
            Degree& operator /= (Real f) { mDeg /= f; return *this; }

            bool operator <  (const Degree& d) const { return mDeg < d.mDeg; }
            bool operator <= (const Degree& d) const { return mDeg <= d.mDeg; }
            bool operator == (const Degree& d) const { return mDeg == d.mDeg; }
            bool operator != (const Degree& d) const { return mDeg != d.mDeg; }
            bool operator >= (const Degree& d) const { return mDeg >= d.mDeg; }
            bool operator >  (const Degree& d) const { return mDeg > d.mDeg; }

            inline friend std::ostream& operator <<
                (std::ostream& o, const Degree& v)
            {
                o << "Degree(" << v.valueDegrees() << ")";
                return o;
            }
        };

        class Angle
        {
            Real mAngle;
        public:
            explicit Angle(Real angle) : mAngle(angle) {}
            operator Radian() const;
            operator Degree() const;
        };

        // these functions could not be defined within the class definition of class
        // Radian because they required class Degree to be defined
        inline Radian::Radian(const Degree& d) : mRad(d.valueRadians()) {
        }
        inline Radian& Radian::operator = (const Degree& d) {
            mRad = d.valueRadians(); return *this;
        }
        inline Radian Radian::operator + (const Degree& d) const {
            return Radian(mRad + d.valueRadians());
        }
        inline Radian& Radian::operator += (const Degree& d) {
            mRad += d.valueRadians();
            return *this;
        }
        inline Radian Radian::operator - (const Degree& d) const {
            return Radian(mRad - d.valueRadians());
        }
        inline Radian& Radian::operator -= (const Degree& d) {
            mRad -= d.valueRadians();
            return *this;
        }

        class Math
        {
        public:
            enum AngleUnit
            {
                AU_DEGREE,
                AU_RADIAN
            };

        protected:
            // angle units used by the api
            static AngleUnit msAngleUnit;

            static int mTrigTableSize;

            static Real mTrigTableFactor;
            static Real* mSinTable;
            static Real* mTanTable;

            void buildTrigTables();

            static float SinTable(float fValue);
            static float TanTable(float fValue);

            
        public:
            Math(unsigned int trigTableSize = 4096);

            ~Math();

            static inline int IAbs(int iValue) { return (iValue >= 0 ? iValue : -iValue); }
            static inline int ICeil(float fValue) { return int(ceil(fValue)); }
            static inline int IFloor(float fValue) { return int(floor(fValue)); }
            static int ISign(int iValue) {
                return (iValue > 0 ? +1 : (iValue < 0 ? -1 : 0));
            }

            static inline Real Abs(Real fValue) { return Real(fabs(fValue)); }
            static inline Degree Abs(const Degree& dValue) { return Degree(fabs(dValue.valueDegrees())); }
            static inline Radian Abs(const Radian& rValue) { return Radian(fabs(rValue.valueRadians())); }
            static Radian ACos(Real fValue);
            static Radian ASin(Real fValue);
            static inline Radian ATan(Real fValue) { return Radian(atan(fValue)); }
            static inline Radian ATan2(Real fY, Real fX) { return Radian(atan2(fY, fX)); }
            static inline Real Ceil(Real fValue) { return Real(ceil(fValue)); }
            static inline bool isNaN(Real f)
            {
                // std::isnan() is C99, not supported by all compilers
                // However NaN always fails this next test, no other number does.
                return f != f;
            }

            static inline Real Cos(const Radian& fValue, bool useTables = false) {
                return (!useTables) ? Real(cos(fValue.valueRadians())) : SinTable(fValue.valueRadians() + HALF_PI);
            }
            static inline Real Cos(Real fValue, bool useTables = false) {
                return (!useTables) ? Real(cos(fValue)) : SinTable(fValue + HALF_PI);
            }

            static inline Real Exp(Real fValue) { return Real(exp(fValue)); }

            static inline Real Floor(Real fValue) { return Real(floor(fValue)); }

            static inline Real Log(Real fValue) { return Real(log(fValue)); }

            static const Real LOG2;

            static inline Real Log2(Real fValue) { return Real(log(fValue) / LOG2); }

            static inline Real LogN(Real base, Real fValue) { return Real(log(fValue) / log(base)); }

            static inline Real Pow(Real fBase, Real fExponent) { return Real(pow(fBase, fExponent)); }

            static Real Sign(Real fValue)
            {
                if (fValue > 0.0)
                    return 1.0;
                if (fValue < 0.0)
                    return -1.0;
                return 0.0;
            }

            static inline Radian Sign(const Radian& rValue)
            {
                return Radian(Sign(rValue.valueRadians()));
            }
            static inline Degree Sign(const Degree& dValue)
            {
                return Degree(Sign(dValue.valueDegrees()));
            }

            static inline Real Sin(const Radian& fValue, bool useTables = false) {
                return (!useTables) ? Real(sin(fValue.valueRadians())) : SinTable(fValue.valueRadians());
            }
            static inline Real Sin(Real fValue, bool useTables = false) {
                return (!useTables) ? Real(sin(fValue)) : SinTable(fValue);
            }

            static inline Real Sqr(Real fValue) { return fValue * fValue; }

            static inline Real Sqrt(Real fValue) { return Real(sqrt(fValue)); }

            static inline Radian Sqrt(const Radian& fValue) { return Radian(sqrt(fValue.valueRadians())); }

            static inline Degree Sqrt(const Degree& fValue) { return Degree(sqrt(fValue.valueDegrees())); }

            static Real InvSqrt(Real fValue) {
                return Real(1.) / std::sqrt(fValue);
            }

            static Real UnitRandom();  // in [0,1]

            static Real RangeRandom(Real fLow, Real fHigh) { // in [fLow,fHigh]
                return (fHigh - fLow) * UnitRandom() + fLow;
            }  

            static Real SymmetricRandom() { // in [-1,1]
                return 2.0f * UnitRandom() - 1.0f;
            }  

            static inline Real Tan(const Radian& fValue, bool useTables = false) {
                return (!useTables) ? Real(tan(fValue.valueRadians())) : TanTable(fValue.valueRadians());
            }
            static inline Real Tan(Real fValue, bool useTables = false) {
                return (!useTables) ? Real(tan(fValue)) : TanTable(fValue);
            }

            static inline Real DegreesToRadians(Real degrees) { return degrees * fDeg2Rad; }
            static inline Real RadiansToDegrees(Real radians) { return radians * fRad2Deg; }

            static void setAngleUnit(AngleUnit unit);
            static AngleUnit getAngleUnit(void);

            static float AngleUnitsToRadians(float units);
            static float RadiansToAngleUnits(float radians);
            static float AngleUnitsToDegrees(float units);
            static float DegreesToAngleUnits(float degrees);

            static bool pointInTri2D(const glm::vec2& p, const glm::vec2& a,
                const glm::vec2& b, const glm::vec2& c);

            static bool pointInTri3D(const glm::vec3& p, const glm::vec3& a,
                const glm::vec3& b, const glm::vec3& c, const glm::vec3& normal);

            static std::pair<bool, Real> intersects(const Ray& ray, const Plane& plane);

            static std::pair<bool, Real> intersects(const Ray& ray, const Sphere& sphere,
                bool discardInside = true);

            static std::pair<bool, Real> intersects(const Ray& ray, const AxisAlignedBox& box);

            static bool intersects(const Ray& ray, const AxisAlignedBox& box,
                Real* d1, Real* d2);

            static std::pair<bool, Real> intersects(const Ray& ray, const glm::vec3& a,
                const glm::vec3& b, const glm::vec3& c, const glm::vec3& normal,
                bool positiveSide = true, bool negativeSide = true);

            static std::pair<bool, Real> intersects(const Ray& ray, const glm::vec3& a,
                const glm::vec3& b, const glm::vec3& c,
                bool positiveSide = true, bool negativeSide = true);

            static bool intersects(const Sphere& sphere, const AxisAlignedBox& box);

            static bool intersects(const Plane& plane, const AxisAlignedBox& box);

            // TODO:
            /*static std::pair<bool, Real> intersects(
                const Ray& ray, const vector<Plane>::type& planeList,
                bool normalIsOutside);*/

                /*
                static std::pair<bool, Real> intersects(
                    const Ray& ray, const list<Plane>::type& planeList,
                    bool normalIsOutside);*/

            static bool intersects(const Sphere& sphere, const Plane& plane);

            static bool RealEqual(Real a, Real b,
                Real tolerance = std::numeric_limits<Real>::epsilon()) {
                return std::abs(b - a) <= tolerance;
            }

            static glm::vec3 calculateTangentSpaceVector(
                const glm::vec3& position1, const glm::vec3& position2, const glm::vec3& position3,
                Real u1, Real v1, Real u2, Real v2, Real u3, Real v3);

            static glm::mat4x4 buildReflectionMatrix(const Plane& p);
            static glm::vec4 calculateFaceNormal(const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3);
            static glm::vec3 calculateBasicFaceNormal(const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3);
            static glm::vec4 calculateFaceNormalWithoutNormalize(const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3);
            static glm::vec3 calculateBasicFaceNormalWithoutNormalize(const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3);

            static Real gaussianDistribution(Real x, Real offset = 0.0f, Real scale = 1.0f);

            template <typename T>
            static T Clamp(T val, T minval, T maxval)
            {
                assert(minval < maxval && "Invalid clamp range");
                return std::max(std::min(val, maxval), minval);
            }

            static glm::mat4x4 makeViewMatrix(const glm::vec3& position, const glm::quat& orientation,
                const glm::mat4x4* reflectMatrix = 0);

            static Real boundingRadiusFromAABB(const AxisAlignedBox& aabb);



            static const Real POS_INFINITY;
            static const Real NEG_INFINITY;
            static const Real PI;
            static const Real TWO_PI;
            static const Real HALF_PI;
            static const Real fDeg2Rad;
            static const Real fRad2Deg;

        };

        // these functions must be defined down here, because they rely on the
        // angle unit conversion functions in class Math:

        inline Real Radian::valueDegrees() const
        {
            return Math::RadiansToDegrees(mRad);
        }

        inline Real Radian::valueAngleUnits() const
        {
            return Math::RadiansToAngleUnits(mRad);
        }

        inline Real Degree::valueRadians() const
        {
            return Math::DegreesToRadians(mDeg);
        }

        inline Real Degree::valueAngleUnits() const
        {
            return Math::DegreesToAngleUnits(mDeg);
        }

        inline Angle::operator Radian() const
        {
            return Radian(Math::AngleUnitsToRadians(mAngle));
        }

        inline Angle::operator Degree() const
        {
            return Degree(Math::AngleUnitsToDegrees(mAngle));
        }

        inline Radian operator * (Real a, const Radian& b)
        {
            return Radian(a * b.valueRadians());
        }

        inline Radian operator / (Real a, const Radian& b)
        {
            return Radian(a / b.valueRadians());
        }

        inline Degree operator * (Real a, const Degree& b)
        {
            return Degree(a * b.valueDegrees());
        }

        inline Degree operator / (Real a, const Degree& b)
        {
            return Degree(a / b.valueDegrees());
        }

    }
}
