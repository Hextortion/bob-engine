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

Box<float> Triangle::getBoundingBox() const
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

// float edgeFunction(const Vec3<float>& a, const Vec3<float>& b, const Vec3<float>& c)
// {
//     return (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0]);
// }

float edgeFunction(const Vec3<float>& v0, const Vec3<float>& v1, const Vec3<float>& p)
{
    return (v1.y - v0.y) * (p.x - v0.x) - (v1.x - v0.x) * (p.y - v0.y);
}

optional<Box<int>>
imageIntersection(Box<float> box, int width, int height)
{
    if (box.xmin >= width - 1 || box.xmax < 0 ||
        box.ymin >= height - 1 || box.ymax < 0)
    {
        return nullopt;
    }

    return Box<int> {
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

using LineClipCode = int;

const LineClipCode LINE_CLIP_INSIDE = 0;
const LineClipCode LINE_CLIP_LEFT = 1;
const LineClipCode LINE_CLIP_RIGHT = 2;
const LineClipCode LINE_CLIP_BOTTOM = 4;
const LineClipCode LINE_CLIP_TOP = 8;

LineClipCode computeOutCode(const Box<int>& r, Vec2<int> p)
{
    LineClipCode code;

    code = LINE_CLIP_INSIDE;

    if (p.x < r.xmin) {
        code |= LINE_CLIP_LEFT;
    } else if (p.x > r.xmax) {
        code |= LINE_CLIP_RIGHT;
    }

    if (p.y < r.ymin) {
        code |= LINE_CLIP_BOTTOM;
    } else if (p.y > r.ymax) {
        code |= LINE_CLIP_TOP;
    }

    return code;
}

// Cohen–Sutherland clipping algorithm
optional<Line2D<int>>
clipLineAgainstRectangle(const Box<int>& r, Line2D<int> line)
{
    LineClipCode code0 = computeOutCode(r, line.start);
    LineClipCode code1 = computeOutCode(r, line.end);

    while (true) {
        if (!(code0 | code1)) {
            return line;
        } else if (code0 & code1) {
            return nullopt;
        } else {
            int x, y;
            LineClipCode code = code0 ? code0 : code1;

            if (code & LINE_CLIP_TOP) {
                x = line.start.x + (line.end.x - line.start.x) * (r.ymax - line.start.y) / (line.end.y - line.start.y);
                y = r.ymax;
            } else if (code & LINE_CLIP_BOTTOM) {
                x = line.start.x + (line.end.x - line.start.x) * (r.ymin - line.start.y) / (line.end.y - line.start.y);
                y = r.ymin;
            } else if (code & LINE_CLIP_RIGHT) {
                y = line.start.y + (line.end.y - line.start.y) * (r.xmax - line.start.x) / (line.end.x - line.start.x);
                x = r.xmax;
            } else if (code & LINE_CLIP_LEFT) {
                y = line.start.y + (line.end.y - line.start.y) * (r.xmin - line.start.x) / (line.end.x - line.start.x);
                x = r.xmin;
            }

            if (code == code0) {
                line.start.x = x;
                line.start.y = y;
                code0 = computeOutCode(r, line.start);
            } else {
                line.end.x = x;
                line.end.y = y;
                code1 = computeOutCode(r, line.end);
            }
        }
    }
}

void Engine::drawLine(
    Line2D<int> line,
    uint8_t r, uint8_t g, uint8_t b)
{
    auto clipped = clipLineAgainstRectangle(Box<int>{0, width, 0, height}, line);

     if (clipped) {
        int dx = clipped->end.x - clipped->start.x;
        int dy = clipped->end.y - clipped->start.y;
        int dLong = abs(dx);
        int dShort = abs(dy);

        int offsetLong = dx > 0 ? 1 : -1;
        int offsetShort = dy > 0 ? width : -width;

        if (dLong < dShort) {
            swap(dShort, dLong);
            swap(offsetShort, offsetLong);
        }

        int D = 2 * dShort - dLong;
        int index = clipped->start.y * width + clipped->start.x;
        const int offset[] = { offsetLong, offsetLong + offsetShort };
        const int incD[] = { 2 * dShort, 2 * dShort - 2 * dLong };

        for (int i = 0; i <= dLong; i++)  {
            if (index >= 0 && index < width * height)
                mem[index] = (r << 16) | (g << 8) | b;
            const int DGreaterThan0 = D > 0;
            index += offset[DGreaterThan0];
            D += incD[DGreaterThan0];
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

    drawLine({{x0, y0}, {x1, y1}}, r, g, b);
    drawLine({{x2, y2}, {x1, y1}}, r, g, b);
    drawLine({{x0, y0}, {x2, y2}}, r, g, b);
}

void Engine::drawTriangle(Triangle tri, bool clipped)
{
    float lightFactor = max(0.0f, lightSource.dot(tri.normal));
    uint8_t luminance = static_cast<uint8_t>(floor(lightFactor * 256.0f));

    tri = Triangle {
        worldToRasterSpace(tri.v[0], camera, width, height),
        worldToRasterSpace(tri.v[1], camera, width, height),
        worldToRasterSpace(tri.v[2], camera, width, height),
    };

    auto boundingBox = imageIntersection(tri.getBoundingBox(), width, height);

    if (boundingBox) {
        tri.v[0].z = 1 / tri.v[0].z;
        tri.v[1].z = 1 / tri.v[1].z;
        tri.v[2].z = 1 / tri.v[2].z;

        float A01 = tri.v[1].y - tri.v[0].y;
        float A12 = tri.v[2].y - tri.v[1].y;
        float A20 = tri.v[0].y - tri.v[2].y;

        float B01 = tri.v[0].x - tri.v[1].x;
        float B12 = tri.v[1].x - tri.v[2].x;
        float B20 = tri.v[2].x - tri.v[0].x;

        Vec3<float> centerPixel(boundingBox->xmin + 0.5, boundingBox->ymin + 0.5, 0);
        float w0Row = edgeFunction(tri.v[1], tri.v[2], centerPixel);
        float w1Row = edgeFunction(tri.v[2], tri.v[0], centerPixel);
        float w2Row = edgeFunction(tri.v[0], tri.v[1], centerPixel);
        float area = edgeFunction(tri.v[0], tri.v[1], tri.v[2]);

        for (int y = boundingBox->ymin; y <= boundingBox->ymax; ++y) {
            float w0 = w0Row;
            float w1 = w1Row;
            float w2 = w2Row;

            for (int x = boundingBox->xmin; x <= boundingBox->xmax; ++x) {
                // float w0 = edgeFunction(tri.v[1], tri.v[2], centerPixel);
                // float w1 = edgeFunction(tri.v[2], tri.v[0], centerPixel);
                // float w2 = edgeFunction(tri.v[0], tri.v[1], centerPixel);

                if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
                    // w0 /= area; w1 /= area; w2 /= area;
                    float z = area / (tri.v[2].z * w0 + tri.v[1].z * w1 + tri.v[0].z * w2);

                    if (z < depthBuffer[y * width + x]) {
                        depthBuffer[y * width + x] = z;
                        mem[y * width + x] = (luminance << 16) | (luminance << 8) | luminance;
                    }
                }

                w0 += A12;
                w1 += A20;
                w2 += A01;
            }

            w0Row += B12;
            w1Row += B20;
            w2Row += B01;
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
