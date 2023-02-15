#pragma once

// Original CSG.JS library by Evan Wallace (http://madebyevan.com), under the MIT license.
// GitHub: https://github.com/evanw/csg.js/
//
// C++ port by Tomasz Dabrowski (http://28byteslater.com), under the MIT license.
// GitHub: https://github.com/dabroz/csgjscpp-cpp/
//
// Constructive Solid Geometry (CSG) is a modeling technique that uses Boolean
// operations like union and intersection to combine 3D solids. This library
// implements CSG operations on meshes elegantly and concisely using BSP trees,
// and is meant to serve as an easily understandable implementation of the
// algorithm. All edge cases involving overlapping coplanar polygons in both
// solids are correctly handled.
//
// modified by dazza - 200421

#include <algorithm>
#include <memory>

#define _USE_MATH_DEFINES
#include <math.h>
#include <cstdint>

#if !defined(CSGJSCPP_REAL)
#define CSGJSCPP_REAL float
#endif

#if !defined(CSGJSCPP_VECTOR)
#include <vector>
#define CSGJSCPP_VECTOR std::vector
#endif

#if !defined(CSGJSCPP_DEQUE)
#include <deque>
#define CSGJSCPP_DEQUE std::deque
#endif

#if !defined(CSGJSCPP_SWAP)
#define CSGJSCPP_SWAP std::swap
#endif

#if !defined(CSGJSCPP_REVERSE)
#define CSGJSCPP_REVERSE std::reverse
#endif

#if !defined(CSGJSCPP_PAIR)
#define CSGJSCPP_PAIR std::pair
#endif

#if !defined(CSGJSCPP_MAKEPAIR)
#define CSGJSCPP_MAKEPAIR std::make_pair
#endif

#if !defined(CSGJSCPP_UNIQUEPTR)
#define CSGJSCPP_UNIQUEPTR std::unique_ptr
#endif

#if !defined(CSGJSCPP_MAP)
#include <map>
#define CSGJSCPP_MAP std::map
#endif

#if !defined(CSGJSCPP_FIND_IF)
#define CSGJSCPP_FIND_IF std::find_if
#endif

#if !defined (CSGJSCPP_INDEX)
#define CSGJSCPP_INDEX uint32_t
#endif

namespace csgjscpp {

// `CSG.Plane.EPSILON` is the tolerance used by `splitPolygon()` to decide if a
// point is on the plane.
const CSGJSCPP_REAL csgjs_EPSILON = 0.0001f;

struct Vector {
    CSGJSCPP_REAL x, y, z;

    Vector() : x(0.0f), y(0.0f), z(0.0f) {
    }
    Vector(CSGJSCPP_REAL x, CSGJSCPP_REAL y, CSGJSCPP_REAL z) : x(x), y(y), z(z) {
    }
};

inline bool approxequal(CSGJSCPP_REAL a, CSGJSCPP_REAL b) {
    return fabs(a - b) < csgjs_EPSILON;
}

inline bool operator==(const Vector &a, const Vector &b) {
    return approxequal(a.x, b.x) && approxequal(a.y, b.y) && approxequal(a.z, b.z);
}

inline bool operator!=(const Vector &a, const Vector &b) {
    return !approxequal(a.x, b.x) || !approxequal(a.y, b.y) || !approxequal(a.z, b.z);
}


// Vector implementation

inline Vector operator+(const Vector &a, const Vector &b) {
    return Vector(a.x + b.x, a.y + b.y, a.z + b.z);
}
inline Vector operator-(const Vector &a, const Vector &b) {
    return Vector(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline Vector operator*(const Vector &a, CSGJSCPP_REAL b) {
    return Vector(a.x * b, a.y * b, a.z * b);
}
inline Vector operator/(const Vector &a, CSGJSCPP_REAL b) {
    return a * ((CSGJSCPP_REAL)1.0 / b);
}
inline CSGJSCPP_REAL dot(const Vector &a, const Vector &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline Vector lerp(const Vector &a, const Vector &b, CSGJSCPP_REAL v) {
    return a + (b - a) * v;
}
inline Vector negate(const Vector &a) {
    return a * -(CSGJSCPP_REAL)1.0;
}
inline CSGJSCPP_REAL length(const Vector &a) {
    return (CSGJSCPP_REAL)sqrt(dot(a, a));
}

inline CSGJSCPP_REAL lengthsquared(const Vector &a) {
    return dot(a, a);
}

inline Vector unit(const Vector &a) {
    return a / length(a);
}
inline Vector cross(const Vector &a, const Vector &b) {
    return Vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

inline Vector operator-(const Vector &a) {
    return Vector(-a.x, -a.y, -a.z);
}

inline uint32_t lerp(uint32_t a, uint32_t b, CSGJSCPP_REAL v) {
    return a + (uint32_t)((b - a) * v);
}

struct Vertex {
    Vector   pos;
};

inline bool operator==(const Vertex &a, const Vertex &b) {
    return a.pos == b.pos;
}

inline bool operator!=(const Vertex &a, const Vertex &b) {
    return a.pos != b.pos;
}


struct Polygon;

// Represents a plane in 3D space.
struct Plane {
    Vector        normal;
    CSGJSCPP_REAL w;

    Plane();
    Plane(const Vector &a, const Vector &b, const Vector &c);

    inline bool ok() const {
        return length(this->normal) > 0.0f;
    }

    inline void flip() {
        this->normal = negate(this->normal);
        this->w *= -1.0f;
    }

    void splitpolygon(const Polygon &poly, CSGJSCPP_VECTOR<Polygon> &coplanarFront,
                       CSGJSCPP_VECTOR<Polygon> &coplanarBack, CSGJSCPP_VECTOR<Polygon> &front,
                       CSGJSCPP_VECTOR<Polygon> &back) const;

    enum Classification { COPLANAR = 0, FRONT = 1, BACK = 2, SPANNING = 3 };
    inline Classification classify(const Vector &p) const {
        CSGJSCPP_REAL  t = dot(normal, p) - this->w;
        Classification c = (t < -csgjs_EPSILON) ? BACK : ((t > csgjs_EPSILON) ? FRONT : COPLANAR);
        return c;
    }
};

// Represents a convex polygon. The vertices used to initialize a polygon must
// be coplanar and form a convex loop. They do not have to be `CSG.Vertex`
// instances but they must behave similarly (duck typing can be used for
// customization).
//
// Each convex polygon has a `shared` property, which is shared between all
// polygons that are clones of each other or were split from the same polygon.
// This can be used to define per-polygon properties (such as surface color).
struct Polygon {
    CSGJSCPP_VECTOR<Vertex> vertices;
    Plane                   plane;

    Polygon();
    Polygon(const CSGJSCPP_VECTOR<Vertex> &list);

    inline void flip() {
        CSGJSCPP_REVERSE(vertices.begin(), vertices.end());
        //for (size_t i = 0; i < vertices.size(); i++)
        //    vertices[i].normal = negate(vertices[i].normal);
        plane.flip();
    }
};

struct Model {

    using Index = CSGJSCPP_INDEX;

    CSGJSCPP_VECTOR<Vertex> vertices;
    CSGJSCPP_VECTOR<Index>  indices;
    unsigned int color;

    Index AddVertex(const Vertex &newv) {
        Index i = 0;
        for (const auto &v : vertices) {
            if (v == newv) {
                return i;
            }
            ++i;
        }
        vertices.push_back(newv);
        return i;
    }
};

// public interface - not super efficient, if you use multiple CSG operations you should
// use BSP trees and convert them into model only once. Another optimization trick is
// replacing model with your own class.

Model csgunion(const Model &a, const Model &b);
Model csgintersection(const Model &a, const Model &b);
Model csgsubtract(const Model &a, const Model &b);

CSGJSCPP_VECTOR<Polygon> csgunion(const CSGJSCPP_VECTOR<Polygon> &a, const CSGJSCPP_VECTOR<Polygon> &b);
CSGJSCPP_VECTOR<Polygon> csgintersection(const CSGJSCPP_VECTOR<Polygon> &a, const CSGJSCPP_VECTOR<Polygon> &b);
CSGJSCPP_VECTOR<Polygon> csgsubtract(const CSGJSCPP_VECTOR<Polygon> &a, const CSGJSCPP_VECTOR<Polygon> &b);

/* API to build a set of polygons representning primatves. */
CSGJSCPP_VECTOR<Polygon> csgpolygon_cube(const Vector &center = {0.0f, 0.0f, 0.0f},
                                          const Vector &dim = {1.0f, 1.0f, 1.0f}, const uint32_t col = 0xFFFFFF);
CSGJSCPP_VECTOR<Polygon> csgpolygon_sphere(const Vector &center = {0.0f, 0.0f, 0.0f}, CSGJSCPP_REAL radius = 1.0f,
                                            const uint32_t col = 0xFFFFFF, int slices = 16, int stacks = 8);
CSGJSCPP_VECTOR<Polygon> csgpolygon_cylinder(const Vector &s = {0.0f, -1.0f, 0.0f},
                                              const Vector &e = {0.0f, 1.0f, 0.0f}, CSGJSCPP_REAL radius = 1.0f,
                                              const uint32_t col = 0xFFFFFF, int slices = 16);

CSGJSCPP_VECTOR<Polygon> csgfixtjunc(const CSGJSCPP_VECTOR<Polygon> &polygons);

Model modelfrompolygons(const CSGJSCPP_VECTOR<Polygon> &polygons);
CSGJSCPP_VECTOR<Polygon> modeltopolygons(const Model &model);

/* API to build models representing primatives */
Model csgmodel_cube(const Vector &center = {0.0f, 0.0f, 0.0f}, const Vector &dim = {1.0f, 1.0f, 1.0f},
                     const uint32_t col = 0xFFFFFF);
Model csgmodel_sphere(const Vector &center = {0.0f, 0.0f, 0.0f}, CSGJSCPP_REAL radius = 1.0f,
                       const uint32_t col = 0xFFFFFF, int slices = 16, int stacks = 8);

Model csgmodel_cylinder(const Vector &s = {0.0f, -1.0f, 0.0f}, const Vector &e = {0.0f, 1.0f, 0.0f},
                         CSGJSCPP_REAL radius = 1.0f, const uint32_t col = 0xFFFFFF, int slices = 16);

} // namespace csgjscpp
