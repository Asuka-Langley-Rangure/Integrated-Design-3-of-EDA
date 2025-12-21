#ifndef COMMON_H
#define COMMON_H

#if defined(_MSC_VER) && !defined(_HAS_STD_BYTE)
// Avoid std::byte vs Windows RPC byte ambiguity under MSVC when using <cstddef>
#define _HAS_STD_BYTE 0
#endif

#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>
#include <assert.h>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <map>
#include <list>
#include <set>
#include "string.h"
#include <algorithm>

// Bring in only the std symbols we actually use to avoid leaking std::byte
// into the global namespace (conflicts with Windows RPC 'byte').
using std::string;
using std::vector;
using std::map;
using std::list;
using std::set;
using std::pair;
using std::cout;
using std::cerr;
using std::endl;
using std::ifstream;
using std::ofstream;
using std::fstream;
using std::min;
using std::max;
#define EPS 1.0E-15 
#define INT_DOWN(a) (int)(a)
#define INT_CONVERT(a) (int)(1.0 * (a) + 0.5f)

inline bool float_equal(float a, float b)
{
    return std::fabs(a - b) < EPS;
}

struct POS_2D // POS means postition which can be used to store coordinates, offsets
{
    float x;
    float y;
    POS_2D() { SetZero(); };
    POS_2D(float _x, float _y)
    {
        x = _x;
        y = _y;
    }
    inline void SetZero()
    {
        x = y = 0.0; 
    }
    friend inline std::ostream &operator<<(std::ostream &os, const POS_2D &pos)
    {
        os << "(" << pos.x << "," << pos.y << ")";
        return os;
    }
};
struct VECTOR_2D
{
    float x;
    float y;
    VECTOR_2D()
    {
        SetZero();
    }
    inline void SetZero()
    {
        x = y = 0.0; //!! 0.0!!!!
    }
    friend inline std::ostream &operator<<(std::ostream &os, const VECTOR_2D &vec)
    {
        os << "[" << vec.x << "," << vec.y << "]"; // [] for vectors and () for pos
        return os;
    }
};

struct VECTOR_2D_INT
{
    int x;
    int y;
    VECTOR_2D_INT()
    {
        SetZero();
    }
    inline void SetZero()
    {
        x = y = 0; //!! 0.0!!!!
    }
    friend inline std::ostream &operator<<(std::ostream &os, const VECTOR_2D_INT &vec)
    {
        os << "[" << vec.x << "," << vec.y << "]"; // [] for vectors and () for pos
        return os;
    }
};

struct VECTOR_3D
{
    float x;
    float y;
    float z;
    VECTOR_3D()
    {
        SetZero();
    }
    inline void SetZero()
    {
        x = y = z = 0.0; //!! 0.0!!!!
    }
    friend inline std::ostream &operator<<(std::ostream &os, const VECTOR_3D &vec)
    {
        os << "[" << vec.x << "," << vec.y << "," << vec.z << "]"; // [] for vectors and () for pos
        return os;
    }
    inline VECTOR_3D operator+(const VECTOR_3D &rhs)
    {
        VECTOR_3D v;
        v.x = this->x + rhs.x;
        v.y = this->y + rhs.y;
        v.z = this->z + rhs.z;
        return v;
    }
    inline VECTOR_3D operator-(const VECTOR_3D &rhs) const
    {
        VECTOR_3D v;
        v.x = this->x - rhs.x;
        v.y = this->y - rhs.y;
        v.z = this->z - rhs.z;
        return v;
    }
    inline VECTOR_3D operator*(float c)
    {
        VECTOR_3D v;
        v.x = this->x * c;
        v.y = this->y * c;
        v.z = this->z * c;
        return v;
    }
    inline float operator*(const VECTOR_3D &rhs) const
    {
        return (this->x * rhs.x + this->y * rhs.y + this->z * rhs.z);
    }
};

struct POS_3D : public VECTOR_3D
{
    // float x;
    // float y;
    // float z;
    POS_3D() { SetZero(); };
    POS_3D(float _x, float _y, float _z)
    {
        x = _x;
        y = _y;
        z = _z;
    }
    inline void SetZero()
    {
        x = y = z = 0.0; //!! 0.0!!!!
    }
    friend inline std::ostream &operator<<(std::ostream &os, const POS_3D &pos)
    {
        os << "(" << pos.x << "," << pos.y << "," << pos.z << ")";
        return os;
    }
};

class CRect
{
public:
    CRect()
    {
        Init();
    }
    void Print()
    {
        cout << "lower left: " << ll << " to upper right: " << ur << "\n";
    }
    void Init()
    {
        ll.SetZero();
        ur.SetZero();
    }
    POS_2D ll; // ll: lower left coor
    POS_2D ur; // ur: upper right coor
    float getWidth()
    {
        float width = ur.x - ll.x;
        assert(width > 0.0);
        return width;
    }
    float getHeight()
    {
        float height = ur.y - ll.y;
        assert(height > 0.0);
        return height;
    }
    POS_2D getCenter()
    {
        POS_2D center = ll;
        center.x += 0.5 * this->getWidth();
        center.y += 0.5 * this->getHeight();
        return center;
    }
    float getArea()
    {
        return getHeight() * getWidth();
    }
    bool inside(POS_2D &point)
    {
        return (point.x >= ll.x) && (point.x <= ur.x) && (point.y >= ll.y) && (point.y <= ur.y);
    }
    friend inline std::ostream &operator<<(std::ostream &os, const CRect &rect)
    {
        os << "CRect Size: " << rect.ur.x - rect.ll.x << "," << rect.ur.y - rect.ll.y << endl;
        return os;
    }
};


inline bool float_less(float a, float b) // return true if a < b
{
    return a - b < -1.0 * EPS;
}

inline double getOverlap(double x1, double x2, double x3, double x4) // two lines: x1->x2 and x3->x4
{
    assert(x1 <= x2);
    assert(x3 <= x4);

    // overlapStart: start point of the overlap line
    double overlapStart = max(x1, x3);
    double overlapEnd = min(x2, x4);

    if (overlapStart >= overlapEnd)
    {
        return 0;
    }
    else
    {
        return (overlapStart - overlapEnd);
    }
}

inline double getOverlapArea(double left1, double bottom1, double right1, double top1,
                             double left2, double bottom2, double right2, double top2)
{
    assert(left1 <= right1);
    assert(bottom1 <= top1);
    assert(left2 <= right2);
    assert(bottom2 <= top2);

    double rangeH;
    rangeH = getOverlap(left1, right1, left2, right2);
    if (rangeH == 0)
        return 0;

    double rangeV;
    rangeV = getOverlap(bottom1, top1, bottom2, top2);
    if (rangeV == 0)
        return 0;

    return (rangeH * rangeV);
}

inline double getOverlapArea_2D(CRect rect1, CRect rect2)
{
    double left1 = rect1.ll.x;
    double bottom1 = rect1.ll.y;
    double right1 = rect1.ur.x;
    double top1 = rect1.ur.y;
    double left2 = rect2.ll.x;
    double bottom2 = rect2.ll.y;
    double right2 = rect2.ur.x;
    double top2 = rect2.ur.y;
    return getOverlapArea(left1, bottom1, right1, top1, left2, bottom2, right2, top2);
}

inline double getOverlapArea_2D(POS_2D ll1, POS_2D ur1, POS_2D ll2, POS_2D ur2)
{
    CRect rect1;
    CRect rect2;

    rect1.ll = ll1;
    rect1.ur = ur1;

    rect2.ll = ll2;
    rect2.ur = ur2;

    return getOverlapArea_2D(rect1, rect2);
}

template <typename T>
float calc_lipschitz_constant(const std::vector<T> &xt, const std::vector<T> &xt_1, const std::vector<T> &dxt, const std::vector<T> &dxt_1)
{
    assert(xt.size() == xt_1.size());
    assert(dxt.size() == dxt_1.size());
    float squared_sum_denominator = 0;
    float squared_sum_numerator = 0;
    for (size_t idx = 0; idx < xt.size(); idx++)
    {
        squared_sum_numerator += (dxt[idx] - dxt_1[idx]) * (dxt[idx] - dxt_1[idx]);
        squared_sum_denominator += (xt[idx] - xt_1[idx]) * (xt[idx] - xt_1[idx]);
    
    }
    return sqrt(squared_sum_numerator) / sqrt(squared_sum_denominator);
}


#endif
