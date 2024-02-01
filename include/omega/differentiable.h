/*
Stripped version of https://github.com/kyrylo-sovailo/BetterDouble
Written by Kyrylo Sovailo
*/
#pragma once

#include <cmath>
#include <limits>
#include <ostream>
#include <istream>
#include <sstream>
#include <string>
#include <Eigen/Core>

//Core
namespace bd
{
    //Predefine
    template<class T, unsigned int N> class Differentiable;

    //Basic arithmetics
    template<class T, unsigned int N> constexpr inline Differentiable<T, N> operator+(const Differentiable<T, N> &a, const Differentiable<T, N> &b) { Differentiable<T, N> v; v.value = a.value + b.value; for (unsigned int i = 0; i < N; ++i) v.derivative[i] = a.derivative[i] + b.derivative[i]; return v; };
    template<class T, unsigned int N> constexpr inline Differentiable<T, N> operator-(const Differentiable<T, N> &a, const Differentiable<T, N> &b) { Differentiable<T, N> v; v.value = a.value - b.value; for (unsigned int i = 0; i < N; ++i) v.derivative[i] = a.derivative[i] - b.derivative[i]; return v; };
    template<class T, unsigned int N> constexpr inline Differentiable<T, N> operator*(const Differentiable<T, N> &a, const Differentiable<T, N> &b) { Differentiable<T, N> v; v.value = a.value * b.value; for (unsigned int i = 0; i < N; ++i) v.derivative[i] = a.value * b.derivative[i] + b.value * a.derivative[i]; return v; };
    template<class T, unsigned int N> constexpr inline Differentiable<T, N> operator/(const Differentiable<T, N> &a, const Differentiable<T, N> &b) { Differentiable<T, N> v; v.value = a.value / b.value; for (unsigned int i = 0; i < N; ++i) v.derivative[i] = (a.derivative[i] * b.value - a.value * b.derivative[i]) / (b.value * b.value); return v; };

    ///Same as double, but with overloaded operators
    ///@tparam T Base type
    ///@tparam N Number of derivatives
    template<class T, unsigned int N>
    class Differentiable
    {
    public:
        //Variables
        T value;
        T derivative[N];

        //Constructors & assignment
        constexpr inline explicit Differentiable()                         noexcept { this->value = 0;           for (unsigned int i = 0; i < N; ++i) derivative[i] = 0; };
        constexpr inline Differentiable(const T &other)                    noexcept { this->value = other;       for (unsigned int i = 0; i < N; ++i) derivative[i] = 0; };
        constexpr inline Differentiable(const Differentiable<T, N> &other) noexcept { this->value = other.value; for (unsigned int i = 0; i < N; ++i) derivative[i] = other.derivative[i]; };

        //Arithmetics
        constexpr inline Differentiable &operator+=(const Differentiable &other) noexcept { for (unsigned int i = 0; i < N; ++i) derivative[i] += other.derivative[i]; return *this; value += other.value; };
        constexpr inline Differentiable &operator-=(const Differentiable &other) noexcept { for (unsigned int i = 0; i < N; ++i) derivative[i] -= other.derivative[i]; return *this; value -= other.value; };
        constexpr inline Differentiable &operator*=(const Differentiable &other) noexcept { for (unsigned int i = 0; i < N; ++i) derivative[i] = derivative[i] * other.value + other.derivative[i] * value; value *= other.value; return *this; };
        constexpr inline Differentiable &operator/=(const Differentiable &other) noexcept { for (unsigned int i = 0; i < N; ++i) derivative[i] = (derivative[i] * other.value - other.derivative[i] * value) / (other.value * other.value); value /= other.value; return *this; };

        //Transformations
        constexpr inline Differentiable operator+() const noexcept { return *this; };
        constexpr inline Differentiable operator-() const noexcept { Differentiable v; v.value = -this->value; for (unsigned int i = 0; i < N; ++i) v.derivative[i] = -derivative[i]; return v; };

        //Cast
        constexpr inline explicit operator T() noexcept { return value; };
    };

    //Comparison
    template<class T, unsigned int N> constexpr inline bool operator==(const Differentiable<T, N> &a, const Differentiable<T, N> &b) { return a.value == b.value; };
    template<class T, unsigned int N> constexpr inline bool operator!=(const Differentiable<T, N> &a, const Differentiable<T, N> &b) { return a.value != b.value; };
    template<class T, unsigned int N> constexpr inline bool operator> (const Differentiable<T, N> &a, const Differentiable<T, N> &b) { return a.value >  b.value; };
    template<class T, unsigned int N> constexpr inline bool operator< (const Differentiable<T, N> &a, const Differentiable<T, N> &b) { return a.value <  b.value; };
    template<class T, unsigned int N> constexpr inline bool operator>=(const Differentiable<T, N> &a, const Differentiable<T, N> &b) { return a.value >= b.value; };
    template<class T, unsigned int N> constexpr inline bool operator<=(const Differentiable<T, N> &a, const Differentiable<T, N> &b) { return a.value <= b.value; };

    //Trigonometric functions
    template<class T, unsigned int N> constexpr inline Differentiable<T, N> cos  (const Differentiable<T, N> &x) noexcept { Differentiable<T, N> v; v.value = std::cos  (x.value); for (unsigned int i = 0; i < N; ++i) v.derivative[i] = -x.derivative[i] * std::sin(x.value);       return v; };
    template<class T, unsigned int N> constexpr inline Differentiable<T, N> sin  (const Differentiable<T, N> &x) noexcept { Differentiable<T, N> v; v.value = std::sin  (x.value); for (unsigned int i = 0; i < N; ++i) v.derivative[i] =  x.derivative[i] * std::cos(x.value);       return v; };
    template<class T, unsigned int N> constexpr inline Differentiable<T, N> tan  (const Differentiable<T, N> &x) noexcept { Differentiable<T, N> v; v.value = std::tan  (x.value); for (unsigned int i = 0; i < N; ++i) v.derivative[i] =  x.derivative[i] * (v.value + 1);           return v; };
    template<class T, unsigned int N> constexpr inline Differentiable<T, N> acos (const Differentiable<T, N> &x) noexcept { Differentiable<T, N> v; v.value = std::acos (x.value); for (unsigned int i = 0; i < N; ++i) v.derivative[i] = -x.derivative[i] / sqrt(1 - x.value);       return v; };
    template<class T, unsigned int N> constexpr inline Differentiable<T, N> asin (const Differentiable<T, N> &x) noexcept { Differentiable<T, N> v; v.value = std::asin (x.value); for (unsigned int i = 0; i < N; ++i) v.derivative[i] =  x.derivative[i] / sqrt(1 - x.value);       return v; };
    template<class T, unsigned int N> constexpr inline Differentiable<T, N> atan (const Differentiable<T, N> &x) noexcept { Differentiable<T, N> v; v.value = std::atan (x.value); for (unsigned int i = 0; i < N; ++i) v.derivative[i] =  x.derivative[i] / (1 + x.value * x.value); return v; };

    //Minimum, maximum, difference functions
    template<class T, unsigned int N> constexpr inline Differentiable<T, N> fdim(const Differentiable<T, N> &x, const Differentiable<T, N> &y) noexcept { return (x > y) ? (x - y) : ((Differentiable<T, N>)0); };
    template<class T, unsigned int N> constexpr inline Differentiable<T, N> fmax(const Differentiable<T, N> &x, const Differentiable<T, N> &y) noexcept { if (std::isnan(x.value)) return y; if (std::isnan(y.value)) return x; return (x > y) ? (x) : (y); };
    template<class T, unsigned int N> constexpr inline Differentiable<T, N> fmin(const Differentiable<T, N> &x, const Differentiable<T, N> &y) noexcept { if (std::isnan(x.value)) return y; if (std::isnan(y.value)) return x; return (x < y) ? (x) : (y); };
    
    //Other functions
    template<class T, unsigned int N> constexpr inline Differentiable<T, N> fabs(const Differentiable<T, N> &x) noexcept { Differentiable<T, N> v; v.value = std::fabs(x.value); for (unsigned int i = 0; i < N; ++i) v.derivative[i] = ((x.value == 0) ? (std::numeric_limits<T>::quiet_NaN()) : ((x.value > 0) ? (x.derivative[i]) : (-x.derivative[i]))); return v; };
    template<class T, unsigned int N> constexpr inline Differentiable<T, N> abs (const Differentiable<T, N> &x) noexcept { Differentiable<T, N> v; v.value = std::abs (x.value); for (unsigned int i = 0; i < N; ++i) v.derivative[i] = ((x.value == 0) ? (std::numeric_limits<T>::quiet_NaN()) : ((x.value > 0) ? (x.derivative[i]) : (-x.derivative[i]))); return v; };
    template<class T, unsigned int N> constexpr inline Differentiable<T, N> fma (const Differentiable<T, N> &x, const Differentiable<T, N> &y, const Differentiable<T, N> &z) noexcept { return x * y + z; };

    //Classification macro / functions
    template<class T, unsigned int N> constexpr inline int  fpclassify(const Differentiable<T, N> &x) noexcept { return std::fpclassify(x.value); }
    template<class T, unsigned int N> constexpr inline bool isfinite  (const Differentiable<T, N> &x) noexcept { return std::isfinite  (x.value); }
    template<class T, unsigned int N> constexpr inline bool isinf     (const Differentiable<T, N> &x) noexcept { return std::isinf     (x.value); }
    template<class T, unsigned int N> constexpr inline bool isnan     (const Differentiable<T, N> &x) noexcept { return std::isnan     (x.value); }
    template<class T, unsigned int N> constexpr inline bool isnormal  (const Differentiable<T, N> &x) noexcept { return std::isnormal  (x.value); }
    template<class T, unsigned int N> constexpr inline bool signbit   (const Differentiable<T, N> &x) noexcept { return std::signbit   (x.value); }

    //Comparison macro / functions
    template<class T, unsigned int N> constexpr inline bool isgreater     (const Differentiable<T, N> &x, const Differentiable<T, N> &y) noexcept { return std::isgreater     (x.value, y.value); }
    template<class T, unsigned int N> constexpr inline bool isgreaterequal(const Differentiable<T, N> &x, const Differentiable<T, N> &y) noexcept { return std::isgreaterequal(x.value, y.value); }
    template<class T, unsigned int N> constexpr inline bool isless        (const Differentiable<T, N> &x, const Differentiable<T, N> &y) noexcept { return std::isless        (x.value, y.value); }
    template<class T, unsigned int N> constexpr inline bool islessequal   (const Differentiable<T, N> &x, const Differentiable<T, N> &y) noexcept { return std::islessequal   (x.value, y.value); }
    template<class T, unsigned int N> constexpr inline bool islessgreater (const Differentiable<T, N> &x, const Differentiable<T, N> &y) noexcept { return std::islessgreater (x.value, y.value); }
    template<class T, unsigned int N> constexpr inline bool isunordered   (const Differentiable<T, N> &x, const Differentiable<T, N> &y) noexcept { return std::isunordered   (x.value, y.value); }
}

//STL integration
namespace std
{
    template<class C, class T, unsigned int N> basic_ostream<C> &operator<<(basic_ostream<C> &os, const bd::Differentiable<T, N> &x)
    {
        os << x.value;
        for (unsigned int i = 0; i < N; i++) { os << ' ' << x.derivative[i]; }
        return os;
    }

    template<class C, class T, unsigned int N> basic_istream<C> &operator>>(basic_istream<C> &is, const bd::Differentiable<T, N> &x)
    {
        is >> x.value;
        for (unsigned int i = 0; i < N; i++) { is >> ' ' >> x.derivative[i]; }
        return is;
    }

    template<class T, unsigned int N> std::string to_string(const bd::Differentiable<T, N> &x)
    {
        std::stringstream str;
        str << x.value;
        for (unsigned int i = 0; i < N; i++) { str << ' ' << x.derivative[i]; }
        return str.str();
    }

    template<class T, unsigned int N> std::wstring to_wstring(const bd::Differentiable<T, N> &x)
    {
        std::wstringstream str;
        str << x.value;
        for (unsigned int i = 0; i < N; i++) { str << ' ' << x.derivative[i]; }
        return str.str();
    }
};

//Eigen integration
template<class T, unsigned int N> struct Eigen::NumTraits<bd::Differentiable<T, N>>
{
    typedef bd::Differentiable<T, N> Real;
    typedef bd::Differentiable<T, N> NonInteger;
    typedef bd::Differentiable<T, N> Literal;
    typedef bd::Differentiable<T, N> Nested;

    enum
    {
        IsComplex = 0,
        IsInteger = 0,
        IsSigned = 1,
        ReadCost = 1,
        AddCost = 3,
        MulCost = 3,
        RequireInitialization = 0
    };

    static constexpr inline bd::Differentiable<T, N> epsilon        () noexcept { return (bd::Differentiable<T, N>)std::numeric_limits<T>::epsilon(); }
    static constexpr inline bd::Differentiable<T, N> dummy_precision() noexcept { return (bd::Differentiable<T, N>)std::numeric_limits<T>::epsilon(); }
    static constexpr inline bd::Differentiable<T, N> highest        () noexcept { return (bd::Differentiable<T, N>)std::numeric_limits<T>::infinity(); }
    static constexpr inline bd::Differentiable<T, N> lowest         () noexcept { return (bd::Differentiable<T, N>)-std::numeric_limits<T>::infinity(); }
    static constexpr inline int                      digits         () noexcept { return std::numeric_limits<T>::digits; }
    static constexpr inline int                      digits10       () noexcept { return std::numeric_limits<T>::digits10; }
    static constexpr inline int                      min_exponent   () noexcept { return std::numeric_limits<T>::min_exponent; }
    static constexpr inline int                      max_exponent   () noexcept { return std::numeric_limits<T>::max_exponent; }
    static constexpr inline bd::Differentiable<T, N> infinity       () noexcept { return (bd::Differentiable<T, N>)std::numeric_limits<T>::infinity(); }
    static constexpr inline bd::Differentiable<T, N> quiet_NaN      () noexcept { return (bd::Differentiable<T, N>)std::numeric_limits<T>::quiet_NaN(); }
};

//Omega integration
namespace omega
{
    typedef bd::Differentiable<double, 1> ddouble;
    typedef bd::Differentiable<double, 3> ddouble3;
};