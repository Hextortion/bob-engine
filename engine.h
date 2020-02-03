#ifndef RENDER_H
#define RENDER_H

#include <fstream>
#include <cstdint>
#include <vector>
#include <utility>
#include <cmath>

extern const double gPi;

template <typename T>
struct Vec2
{
    Vec2() : x(T(0)), y(T(0)) {}
    Vec2(T x, T y) : x(T(x)), y(T(y)) {}
    T x, y;
};

template <typename T>
struct Vec3
{
    Vec3() : x(T(0)), y(T(0)), z(T(0)) {}
    Vec3(T x, T y, T z) : x(T(x)), y(T(y)), z(T(z)) {}

    // Not entirely sure this is defined behavior in C++
    const T& operator[](int i) const { return (&x)[i]; }
    T& operator[](int i) { return (&x)[i]; }

    Vec3 operator-(const Vec3& other) const
    {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }

    Vec3 operator+(const Vec3& other) const
    {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }

    Vec3 operator/(const T& r) const
    {
        return Vec3(x / r, y / r, z / r);
    }

    Vec3 operator*(const T& r) const
    {
        return Vec3(x * r, y * r, z * r);
    }

    Vec3& operator-=(const Vec3& other)
    {
        x -= other.x; y -= other.y; z -= other.z;
        return *this;
    }

    Vec3& operator+=(const Vec3& other)
    {
        x += other.x; y += other.y; z += other.z;
        return *this;
    }

    Vec3& operator/=(const T &r)
    {
        x /= r; y /= r; z /= r;
        return *this;
    }

    Vec3& operator*=(const T &r)
    {
        x *= r, y *= r, z *= r;
        return *this;
    }

    Vec3 cross(const Vec3<T>& other) const
    {
        return Vec3<T>{
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }

    T dot(const Vec3<T>& other) const
    {
        return x * other.x + y * other.y + z * other.z;
    }

    T norm() const { return x * x + y * y + z * z; }
    T length() const { return sqrt(norm()); }

    Vec3& normalize()
    {
        T n = norm();
        if (n > 0) {
            T factor = 1 / sqrt(n);
            x *= factor; y *= factor; z *= factor;
        }

        return *this;
    }

    T x, y, z;
};

template <typename T>
std::ostream& operator<<(std::ostream& os, const Vec3<T>& v)
{
    os << '[' << v[0] << ", " << v[1] << ", " << v[2] << ']';
    return os;
}

template <typename T>
struct Mat44
{
    T x[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

    const T* operator[](int i) const { return x[i]; }
    T* operator[](int i) { return x[i]; }

    template<typename S>
    Vec3<S> multVecMatrix(const Vec3<S>& src) const
    {
        S a, b, c, w;
        a = src[0] * x[0][0] + src[1] * x[1][0] + src[2] * x[2][0] + x[3][0];
        b = src[0] * x[0][1] + src[1] * x[1][1] + src[2] * x[2][1] + x[3][1];
        c = src[0] * x[0][2] + src[1] * x[1][2] + src[2] * x[2][2] + x[3][2];
        w = src[0] * x[0][3] + src[1] * x[1][3] + src[2] * x[2][3] + x[3][3];
        a /= w; b /= w; c /= w;
        return Vec3<S>{a, b, c};
    }

    Mat44 operator*(const Mat44& other) const
    {
        Mat44 ret;

        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                ret[i][j] = x[i][0] * other[0][j] + x[i][1] * other[1][j] +
                            x[i][2] * other[2][j] + x[i][3] * other[3][j];
            }
        }

        return ret;
    }

    Mat44& operator*=(const Mat44& other)
    {
        Mat44 ret = *this * other;
        std::swap(ret, *this);
        return *this;
    }

    Mat44 inverse() const
    {
        Mat44 s;
        Mat44 t(*this);

        for (int i = 0; i < 3; ++i) {
            int pivot = i;
            T pivotsize = t[i][i];

            if (pivotsize < 0) {
                pivotsize = -pivotsize;
            }

            for (int j = i + 1; j < 4; ++j) {
                T tmp = t[j][i];

                if (tmp < 0) tmp = -tmp;

                if (tmp > pivotsize) {
                    pivot = j;
                    pivotsize = tmp;
                }
            }

            if (pivotsize == 0) {
                // singular matrix
                return Mat44();
            }

            if (pivot != i) {
                for (int j = 0; j < 4; ++j) {
                    std::swap(t[i][j], t[pivot][j]);
                    std::swap(s[i][j], s[pivot][j]);
                }
            }

            for (int j = i + 1; j < 4; ++j) {
                T f = t[j][i] / t[i][i];
                for (int k = 0; k < 4; ++k) {
                    t[j][k] -= f * t[i][k];
                    s[j][k] -= f * s[i][k];
                }
            }
        }

        for (int i = 3; i >= 0; --i) {
            T f = t[i][i];
            if (f == 0) {
                // singular matrix
                return Mat44();
            }

            for (int j = 0; j < 4; ++j) {
                t[i][j] /= f;
                s[i][j] /= f;
            }

            for (int j = 0; j < i; ++j) {
                f = t[j][i];
                for (int k = 0; k < 4; ++k) {
                    t[j][k] -= f * t[i][k];
                    s[j][k] -= f * s[i][k];
                }
            }
        }

        return s;
    }

    Mat44 quickInverse()
    {
        Mat44 matrix;
        matrix[0][0] = x[0][0]; matrix[0][1] = x[1][0]; matrix[0][2] = x[2][0]; matrix[0][3] = 0.0f;
        matrix[1][0] = x[0][1]; matrix[1][1] = x[1][1]; matrix[1][2] = x[2][1]; matrix[1][3] = 0.0f;
        matrix[2][0] = x[0][2]; matrix[2][1] = x[1][2]; matrix[2][2] = x[2][2]; matrix[2][3] = 0.0f;
        matrix[3][0] = -(x[3][0] * matrix[0][0] + x[3][1] * matrix[1][0] + x[3][2] * matrix[2][0]);
        matrix[3][1] = -(x[3][0] * matrix[0][1] + x[3][1] * matrix[1][1] + x[3][2] * matrix[2][1]);
        matrix[3][2] = -(x[3][0] * matrix[0][2] + x[3][1] * matrix[1][2] + x[3][2] * matrix[2][2]);
        matrix[3][3] = 1.0f;
        return matrix;
    }
};

Mat44<float> rotationAboutAxis(Vec3<float> v, float theta);

template <typename T>
struct Box {
    T xmin, xmax, ymin, ymax;
};

template <typename T>
struct Line2D {
    Vec2<T> start, end;
};

struct Triangle {
    Vec3<float> v[3];
    Vec3<float> normal;

    Triangle() {}

    Triangle(Vec3<float> v0, Vec3<float> v1, Vec3<float> v2) : v{v0, v1, v2}
    {
        normal = (v2 - v0).cross(v1 - v0).normalize();
    }

    Box<float> getBoundingBox() const;
};

struct ClipResult {
    bool clipped = false;
    std::vector<Triangle> newTriangles = std::vector<Triangle>(0);
};

struct Plane {
    Vec3<float> pos;
    Vec3<float> normal;
    ClipResult clipTriangle(const Triangle& tri);
    Vec3<float> intersectLine(Vec3<float> start, Vec3<float> end);
    float distance(Vec3<float> v);
};

struct Camera {
    Mat44<float> worldToCamera;
    Mat44<float> cameraToWorld;
    Vec3<float> pos{-20.0f, -20.0f, -20.0f};
    Vec3<float> forwardDir{0.0, 0.0, 1.0};
    Vec3<float> rightDir{1.0, 0.0, 0.0};
    Vec3<float> upDir{0.0, 1.0, 0.0};
    float pitch = 0.f, yaw = 0.f;
    float left, right, top, bottom;
    bool firstMouse = true;

    // Using _ because some shady header nonsense going on somewhere
    float near_, far_;

    void update(float fov, float near, float far, float aspectRatio);

    void updateTranslationRotation(int xpos, int ypos);
};

struct Engine {
    bool mem_valid = false;
    int width;
    int height;
    uint32_t *mem;

    Vec3<float> lightSource = {1, 0, 0};
    Camera camera;
    std::vector<Triangle> tris;
    std::vector<float> depthBuffer;
    bool wireframe = false;

    Engine(int width, int height, float fov);

    void loadObj(const std::string& filename);
    void fillBuffer(uint8_t r, uint8_t g, uint8_t b);
    void drawLine(Line2D<int>, uint8_t r, uint8_t g, uint8_t b);
    void render();
    void drawTriangleWireframe(Triangle& tri, uint8_t r, uint8_t g, uint8_t b);
    void drawTriangle(Triangle tri, bool clipped = false);
};


#endif // ENGINE_H
