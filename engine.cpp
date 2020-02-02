#include <cstdint>
#include <cmath>
#include <utility>
#include <vector>
#include <optional>
#include <sstream>
#include <iostream>
#include <cstring>

#include "engine.h"

using namespace std;

const double gPi = 3.14159265358979323846;

BoundingBox<float> Triangle::getBoundingBox() const
{
    return { min(v[0].x, min(v[1].x, v[2].x)), max(v[0].x, max(v[1].x, v[2].x)),
             min(v[0].y, min(v[1].y, v[2].y)), max(v[0].y, max(v[1].y, v[2].y)) };
}

float Plane::distance(Vec3<float> v)
{
    return (v - pos).dot(normal.normalize());
}

Vec3<float> Plane::intersectLine(Vec3<float> start, Vec3<float> end)
{
    Vec3<float> line = end - start;
    Vec3<float> unit_normal = normal.normalize();
    float a = (pos - start).dot(unit_normal) / line.dot(unit_normal);
    return start + line * a;
}

ClipResult Plane::clipTriangle(const Triangle& tri)
{
    ClipResult result;

    float d[3];
    int outsideIndices[3];
    int insideIndices[3];
    int outsideCount = 0;
    int insideCount = 0;

    for (int i = 0; i < 3; ++i) {
        d[i] = distance(tri.v[i]);
        if (d[i] < 0.0f) {
            outsideIndices[outsideCount] = i;
            ++outsideCount;
        } else {
            insideIndices[insideCount] = i;
            ++insideCount;
        }
    }

    switch (outsideCount) {
        case 0: return result;

        case 1:
        {
            result.clipped = true;
            result.newTriangles.reserve(2);
            Vec3<float> intersect1 = intersectLine(
                tri.v[insideIndices[0]], tri.v[outsideIndices[0]]);
            Vec3<float> intersect2 = intersectLine(
                tri.v[insideIndices[1]], tri.v[outsideIndices[0]]);

            result.newTriangles.push_back(
                Triangle(tri.v[insideIndices[1]], intersect1, intersect2));
            result.newTriangles.push_back(
                Triangle(tri.v[insideIndices[0]], tri.v[insideIndices[1]], intersect1));
            return result;
        }

        case 2:
        {
            result.clipped = true;
            result.newTriangles.reserve(1);
            Vec3<float> intersect1 = intersectLine(
                tri.v[insideIndices[0]], tri.v[outsideIndices[0]]);
            Vec3<float> intersect2 = intersectLine(
                tri.v[insideIndices[0]], tri.v[outsideIndices[1]]);
            result.newTriangles.push_back(
                Triangle(tri.v[insideIndices[0]], intersect1, intersect2));
        }

        case 3: result.clipped = true; return result;
        default: return result;
    }
}

void Camera::update(float hfov, float near, float far, float aspectRatio)
{
    right = tan(hfov / 2 * gPi / 180) / near;
    top = right / aspectRatio;
    left = -right;
    bottom = -top;
    near_ = near;
    far_ = far;
}

void Camera::updateTranslationRotation(int xpos, int ypos)
{
    yaw += xpos * 0.005;
    pitch += ypos * 0.005;

    if (pitch > gPi * 0.99 / 2) {
        pitch = gPi * 0.498;
    } else if (pitch < -gPi * 0.99 / 2) {
        pitch = -gPi * 0.498;
    }

    forwardDir = Vec3<float>{cosf(yaw) * cosf(pitch), sinf(pitch), sinf(yaw) * cosf(pitch)};
    rightDir = Vec3<float>(0.0f, 1.0f, 0.0f).cross(forwardDir).normalize();
    upDir = forwardDir.cross(rightDir);

    cameraToWorld[0][0] = rightDir.x;
    cameraToWorld[0][1] = rightDir.y;
    cameraToWorld[0][2] = rightDir.z;
    cameraToWorld[0][3] = 0.0f;
    cameraToWorld[1][0] = upDir.x;
    cameraToWorld[1][1] = upDir.y;
    cameraToWorld[1][2] = upDir.z;
    cameraToWorld[1][3] = 0.0f;
    cameraToWorld[2][0] = forwardDir.x;
    cameraToWorld[2][1] = forwardDir.y;
    cameraToWorld[2][2] = forwardDir.z;
    cameraToWorld[2][3] = 0.0f;
    cameraToWorld[3][0] = pos.x;
    cameraToWorld[3][1] = pos.y;
    cameraToWorld[3][2] = pos.z;
    cameraToWorld[3][3] = 1.0f;

    worldToCamera = cameraToWorld.quickInverse();
}

float edgeFunction(const Vec3<float>& a, const Vec3<float>& b, const Vec3<float>& c)
{
    return (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0]);
}

optional<BoundingBox<int>>
imageIntersection(BoundingBox<float> box, int width, int height)
{
    if (box.xmin >= width - 1 || box.xmax < 0 ||
        box.ymin >= height - 1 || box.ymax < 0)
    {
        return nullopt;
    }

    return BoundingBox<int> {
        max(0, static_cast<int>(floor(box.xmin))),
        min(width - 1, static_cast<int>(floor(box.xmax))),
        max(0, static_cast<int>(floor(box.ymin))),
        min(height - 1, static_cast<int>(floor(box.ymax)))
    };
}

Mat44<float> rotationAboutAxis(Vec3<float> v, float theta)
{
    Mat44<float> m;
    v.normalize();
    float c = cosf(theta);
    float one_minus_c = 1.0f - c;
    float s = sinf(theta);

    m[0][0] = v[0] * v[0] * one_minus_c + c;
    m[0][1] = v[0] * v[1] * one_minus_c - v[2] * s;
    m[0][2] = v[0] * v[2] * one_minus_c + v[1] * s;

    m[1][0] = v[0] * v[1] * one_minus_c + v[2] * s;
    m[1][1] = v[1] * v[1] * one_minus_c + c;
    m[1][2] = v[1] * v[2] * one_minus_c - v[0] * s;

    m[2][0] = v[0] * v[2] * one_minus_c - v[1] * s;
    m[2][1] = v[1] * v[2] * one_minus_c + v[0] * s;
    m[2][2] = v[2] * v[2] * one_minus_c + c;

    m[3][3] = 1.0f;
    return m;
}

Vec3<float> worldToRasterSpace(
    const Vec3<float>& v,
    const Camera& c,
    int width,
    int height)
{
    // Convert to camera coordinates
    Vec3<float> u = c.worldToCamera.multVecMatrix(v);

    // Convert to screen space
    u.x = c.near_ * u.x / -u.z;
    u.y = c.near_ * u.y / -u.z;
    u.z = -u.z;

    // Convert to NDC space (range [-1, 1])
    u.x = 2 * u.x / (c.right - c.left) - (c.right + c.left) / (c.right - c.left);
    u.y = 2 * u.y / (c.top - c.bottom) - (c.top + c.bottom) / (c.top - c.bottom);

    // Convert to raster space
    u.x = (u.x + 1) / 2 * width;
    u.y = (1 - u.y) / 2 * height;

    return u;
}

void Engine::loadObj(const string& filename)
{
    ifstream ifs(filename);
    vector<Vec3<float>> vertices;
    string line;
    while (getline(ifs, line)) {
        istringstream iss(line);
        char first;
        iss >> first;
        if (first == 'v') {
            float a, b, c;
            if (iss >> a >> b >> c) {
                vertices.push_back({a, b, c});
            }
        } else if (first == 'f') {
            int a, b, c;
            iss >> a >> b >> c;
            tris.push_back({vertices[a - 1], vertices[b - 1], vertices[c - 1]});
        }
    }
}

Engine::Engine(int width, int height, float fov) : width(width), height(height)
{
    float aspectRatio = static_cast<float>(width) / static_cast<float>(height);
    camera.update(fov, 1, 1000, aspectRatio);
    camera.updateTranslationRotation(width / 2, height / 2);
    depthBuffer = vector<float>(width * height, camera.far_);
}

void Engine::drawLine(
    int x0, int x1, int y0, int y1,
    uint8_t r, uint8_t g, uint8_t b)
{
    auto drawPixel = [=, this](int x, int y) {
        if (y >= 0 && x >= 0 && y < height && x < width) {
            mem[y * width + x] = (r << 16) | (g << 8) | b;
        }
    };

    auto plotLineLow = [=](int x0, int y0, int x1, int y1) {
        int dx = x1 - x0;
        int dy = y1 - y0;
        int yi = 1;

        if (dy < 0) {
            yi = -1;
            dy = -dy;
        }

        int D = 2 * dy - dx;
        int y = y0;

        for (int x = x0; x <= x1; ++x) {
            drawPixel(x, y);

            if (D > 0) {
                y += yi;
                D -= 2 * dx;
            }

            D += 2 * dy;
        }
    };

    auto plotLineHigh = [=](int x0, int y0, int x1, int y1) {
        int dx = x1 - x0;
        int dy = y1 - y0;
        int xi = 1;

        if (dx < 0) {
            xi = -1;
            dx = -dx;
        }

        int D = 2 * dx - dy;
        int x = x0;

        for (int y = y0; y <= y1; ++y) {
            drawPixel(x, y);

            if (D > 0) {
                x += xi;
                D -= 2 * dy;
            }

            D += 2 * dx;
        }
    };

    if (abs(y1 - y0) < abs(x1 - x0)) {
        if (x0 > x1) {
            plotLineLow(x1, y1, x0, y0);
        } else {
            plotLineLow(x0, y0, x1, y1);
        }
    } else {
        if (y0 > y1) {
            plotLineHigh(x1, y1, x0, y0);
        } else {
            plotLineHigh(x0, y0, x1, y1);
        }
    }
}

void Engine::drawTriangleWireframe(
    Triangle& tri,
    uint8_t r, uint8_t g, uint8_t b)
{
    int x0, x1, x2, y0, y1, y2;

    x0 = round(tri.v[0].x);
    x1 = round(tri.v[1].x);
    x2 = round(tri.v[2].x);
    y0 = round(tri.v[0].y);
    y1 = round(tri.v[1].y);
    y2 = round(tri.v[2].y);

    drawLine(x0, x1, y0, y1, r, g, b);
    drawLine(x2, x1, y2, y1, r, g, b);
    drawLine(x0, x2, y0, y2, r, g, b);
}

void Engine::drawTriangle(Triangle tri, bool clipped)
{
    float lightFactor = max(0.0f, lightSource.dot(tri.normal));

    tri = Triangle {
        worldToRasterSpace(tri.v[0], camera, width, height),
        worldToRasterSpace(tri.v[1], camera, width, height),
        worldToRasterSpace(tri.v[2], camera, width, height),
    };

    tri.v[0].z = 1 / tri.v[0].z;
    tri.v[1].z = 1 / tri.v[1].z;
    tri.v[2].z = 1 / tri.v[2].z;

    auto boundingBox = imageIntersection(tri.getBoundingBox(), width, height);

    if (boundingBox) {
        float area = edgeFunction(tri.v[0], tri.v[1], tri.v[2]);

        for (int y = boundingBox->ymin; y <= boundingBox->ymax; ++y) {
            for (int x = boundingBox->xmin; x <= boundingBox->xmax; ++x) {
                Vec3<float> centerPixel{x + 0.5f, y + 0.5f, 0};
                float w0 = edgeFunction(tri.v[1], tri.v[2], centerPixel);
                float w1 = edgeFunction(tri.v[2], tri.v[0], centerPixel);
                float w2 = edgeFunction(tri.v[0], tri.v[1], centerPixel);

                if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
                    w0 /= area; w1 /= area; w2 /= area;
                    float z = 1.0 / (tri.v[2].z * w0 + tri.v[1].z * w1 + tri.v[0].z * w2);

                    if (z < depthBuffer[y * width + x]) {
                        depthBuffer[y * width + x] = z;
                        uint8_t luminance = static_cast<uint8_t>(floor(lightFactor * 256.0f));
                        mem[y * width + x] = (luminance << 16) | (luminance << 8) | luminance;
                    }
                }
            }
        }

        if (wireframe) {
            if (clipped) {
                drawTriangleWireframe(tri, 255, 0, 255);
            } else {
                drawTriangleWireframe(tri, 0, 255, 255);
            }
        }
    }
}

void Engine::render()
{
    memset(mem, 0, width * height * sizeof(uint32_t));
    for (auto& e : depthBuffer) e = camera.far_;

    for (const auto& tri : tris) {
        Plane near{camera.pos - camera.forwardDir * camera.near_, camera.forwardDir * -1};
        auto clipResult = near.clipTriangle(tri);
        if (clipResult.clipped) {
            for (auto& clippedTri : clipResult.newTriangles) {
                drawTriangle(clippedTri, true);
            }
        } else {
            drawTriangle(tri);
        }
    }
}
