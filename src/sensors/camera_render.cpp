// camera_render.cpp - a tiny software renderer for the dash-cam picture.
#include "camera_render.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace sensors {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int W = kImageWidth, H = kImageHeight;
constexpr int kHorizon = 165;                                      // pixel row of the horizon
constexpr double kHalfFov = 35.0 * kPi / 180.0;                    // camera sees +-35 degrees
const double kFocal = (W / 2.0) / std::tan(kHalfFov);              // pixels
constexpr double kCameraHeight = 1.3;                              // metres above the road

// ------------------------------------------------------------------------------------------------
// Colours and a simple drawing surface
// ------------------------------------------------------------------------------------------------
struct Colour {
    double r, g, b;
};
Colour mix(Colour a, Colour b, double t) { return {a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t}; }
Colour darker(Colour c, double k) { return {c.r * k, c.g * k, c.b * k}; }

const Colour kHaze{192, 212, 230};   // colour of the far distance
const Colour kTyre{22, 22, 24};
const Colour kTailLight{235, 35, 35};
const Colour kWhite{240, 240, 236};
const Colour kSkin{222, 178, 140};

// How much an object at `distance` metres fades into the haze (0 = not at all).
double fogAt(double distance) { return std::min(0.85, distance / 240.0); }

class Canvas {
public:
    std::vector<uint8_t> rgb = std::vector<uint8_t>((size_t)W * H * 3, 0);   // rows top to bottom

    void pixel(int x, int y, Colour c, double alpha = 1.0) {
        if (x < 0 || x >= W || y < 0 || y >= H) return;
        uint8_t* p = &rgb[((size_t)y * W + x) * 3];
        double v[3] = {c.r, c.g, c.b};
        for (int i = 0; i < 3; i++) p[i] = (uint8_t)std::clamp(p[i] * (1 - alpha) + v[i] * alpha, 0.0, 255.0);
    }
    void rect(double x0, double y0, double x1, double y1, Colour c) {
        int ix0 = (int)std::floor(x0), iy0 = (int)std::floor(y0);
        int ix1 = std::max(ix0, (int)std::ceil(x1) - 1), iy1 = std::max(iy0, (int)std::ceil(y1) - 1);
        for (int y = iy0; y <= iy1; y++)
            for (int x = ix0; x <= ix1; x++) pixel(x, y, c);
    }
    void ellipse(double cx, double cy, double rx, double ry, Colour c, double alpha = 1.0) {
        rx = std::max(rx, 0.6);
        ry = std::max(ry, 0.6);
        for (int y = (int)std::floor(cy - ry); y <= (int)std::ceil(cy + ry); y++)
            for (int x = (int)std::floor(cx - rx); x <= (int)std::ceil(cx + rx); x++) {
                double dx = (x + 0.5 - cx) / rx, dy = (y + 0.5 - cy) / ry;
                if (dx * dx + dy * dy <= 1.0) pixel(x, y, c, alpha);
            }
    }
    void ring(double cx, double cy, double r, double thickness, Colour c) {
        double half = std::max(thickness, 1.0) / 2;
        for (int y = (int)std::floor(cy - r - half); y <= (int)std::ceil(cy + r + half); y++)
            for (int x = (int)std::floor(cx - r - half); x <= (int)std::ceil(cx + r + half); x++)
                if (std::fabs(std::hypot(x + 0.5 - cx, y + 0.5 - cy) - r) <= half) pixel(x, y, c);
    }
    void line(double x0, double y0, double x1, double y1, double thickness, Colour c) {
        double half = std::max(thickness, 1.0) / 2, vx = x1 - x0, vy = y1 - y0, len2 = vx * vx + vy * vy;
        for (int y = (int)std::floor(std::min(y0, y1) - half); y <= (int)std::ceil(std::max(y0, y1) + half); y++)
            for (int x = (int)std::floor(std::min(x0, x1) - half); x <= (int)std::ceil(std::max(x0, x1) + half); x++) {
                double px = x + 0.5, py = y + 0.5;
                double t = len2 > 0 ? std::clamp(((px - x0) * vx + (py - y0) * vy) / len2, 0.0, 1.0) : 0.0;
                if (std::hypot(px - (x0 + t * vx), py - (y0 + t * vy)) <= half) pixel(x, y, c);
            }
    }
    void triangle(double ax, double ay, double bx, double by, double cx, double cy, Colour c) {
        auto side = [](double x0, double y0, double x1, double y1, double px, double py) {
            return (x1 - x0) * (py - y0) - (y1 - y0) * (px - x0);
        };
        for (int y = (int)std::floor(std::min({ay, by, cy})); y <= (int)std::ceil(std::max({ay, by, cy})); y++)
            for (int x = (int)std::floor(std::min({ax, bx, cx})); x <= (int)std::ceil(std::max({ax, bx, cx})); x++) {
                double px = x + 0.5, py = y + 0.5;
                double s0 = side(bx, by, cx, cy, px, py), s1 = side(cx, cy, ax, ay, px, py), s2 = side(ax, ay, bx, by, px, py);
                if ((s0 >= 0 && s1 >= 0 && s2 >= 0) || (s0 <= 0 && s1 <= 0 && s2 <= 0)) pixel(x, y, c);
            }
    }
    // four corners in order
    void quad(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, Colour c) {
        triangle(x0, y0, x1, y1, x2, y2, c);
        triangle(x0, y0, x2, y2, x3, y3, c);
    }
};

// Small repeatable noise (-0.5 .. 0.5) to give the road and grass some texture.
double noise(long long a, long long b) {
    unsigned long long h = (unsigned long long)(a * 73856093LL) ^ (unsigned long long)(b * 19349663LL);
    h ^= h >> 13;
    h *= 0x5bd1e995ULL;
    h ^= h >> 15;
    return (double)(h % 1000) / 1000.0 - 0.5;
}

// ------------------------------------------------------------------------------------------------
// Background
// ------------------------------------------------------------------------------------------------
void drawSky(Canvas& cv, double pan) {
    for (int y = 0; y < kHorizon; y++)
        for (int x = 0; x < W; x++) cv.pixel(x, y, mix({62, 122, 205}, kHaze, (double)y / kHorizon));

    const std::array<double, 3> clouds[] = {{90, 45, 1.0}, {330, 30, 0.8}, {540, 60, 1.2}};   // x, y, size
    for (const auto& c : clouds) {
        double cx = std::fmod(c[0] - pan + 2 * W, (double)W);
        for (double dx : {-28.0, 0.0, 26.0})
            cv.ellipse(cx + dx * c[2], c[1] + (dx == 0 ? -6 : 0), 34 * c[2], 13 * c[2], {250, 252, 255}, 0.55);
    }
    for (int x = 0; x < W; x++) {   // distant hills
        double xx = x + pan;
        double height = 16 + 9 * std::sin(xx * 0.013 + 1.3) + 5 * std::sin(xx * 0.037 + 0.4);
        for (int y = (int)(kHorizon - height); y < kHorizon; y++) cv.pixel(x, y, mix({96, 140, 118}, kHaze, 0.45));
    }
}

// Every pixel below the horizon is a point on the ground: work out where it is in the world and
// colour it as road, lane marking, gravel or grass.
void drawGround(Canvas& cv, const world::Pose& car) {
    const double cosH = std::cos(car.heading), sinH = std::sin(car.heading);
    for (int y = kHorizon; y < H; y++) {
        double ahead = kCameraHeight * kFocal / (y - kHorizon + 0.5);   // metres in front of the camera
        for (int x = 0; x < W; x++) {
            double left = (W / 2.0 - x - 0.5) * ahead / kFocal;
            double wx = car.x + ahead * cosH - left * sinH, wy = car.y + ahead * sinH + left * cosH;
            double across = std::fabs(wy);
            double n = noise((long long)std::floor(wx * 12), (long long)std::floor(wy * 12));

            Colour c;
            if (across < world::World::kRoadHalfWidth) {
                bool edgeLine = across > world::World::kRoadHalfWidth - 0.3;
                bool laneDash = std::fabs(across - 2.75) < 0.09 && std::fmod(wx + 1e6, 9.0) < 4.0;
                c = (edgeLine || laneDash) ? Colour{232, 232, 228} : Colour{82 + 7 * n, 84 + 7 * n, 90 + 7 * n};
            } else if (across < world::World::kRoadHalfWidth + 0.8) {
                c = {128 + 14 * n, 118 + 14 * n, 100 + 14 * n};   // gravel shoulder
            } else {
                bool stripe = std::fmod(wx + 1e6, 10.0) < 5.0;     // mowed grass
                c = stripe ? Colour{82 + 14 * n, 142 + 14 * n, 70} : Colour{72 + 14 * n, 128 + 14 * n, 62};
            }
            cv.pixel(x, y, mix(c, kHaze, fogAt(ahead)));
        }
    }
}

void drawBonnet(Canvas& cv) {
    cv.ellipse(W / 2.0, H + 52, 430, 78, {30, 33, 38});
    cv.ellipse(W / 2.0, H + 56, 400, 70, {40, 44, 50});
}

// ------------------------------------------------------------------------------------------------
// Obstacles. Each function draws one object standing at screen position (x, ground) where
// s = pixels per metre at its distance. All sizes below are in metres. Returns its bounding box.
// ------------------------------------------------------------------------------------------------
struct Box {
    double x0, y0, x1, y1;
};

struct Brush {   // colours an object, fading it into the haze with distance
    double fog;
    Colour operator()(Colour c) const { return mix(c, kHaze, fog); }
};

void shadow(Canvas& cv, double x, double ground, double width, double s, double fog) {
    cv.ellipse(x, ground, width, 0.12 * s, {0, 0, 0}, 0.35 * (1 - fog));
}

Box drawCar(Canvas& cv, int id, double x, double g, double s, Brush paint) {
    const Colour colours[] = {{205, 40, 45}, {40, 95, 200}, {205, 205, 210}, {245, 245, 245}, {40, 40, 46}, {30, 145, 95}};
    Colour body = paint(colours[id % 6]);
    double hw = 0.95 * s;   // half width
    shadow(cv, x, g, hw * 1.2, s, paint.fog);
    cv.rect(x - hw + 0.05 * s, g - 0.40 * s, x - hw + 0.42 * s, g, paint(kTyre));                 // wheels
    cv.rect(x + hw - 0.42 * s, g - 0.40 * s, x + hw - 0.05 * s, g, paint(kTyre));
    cv.rect(x - hw, g - 0.98 * s, x + hw, g - 0.30 * s, body);                                      // body
    cv.quad(x - hw * 0.92, g - 0.98 * s, x + hw * 0.92, g - 0.98 * s,                              // roof
            x + hw * 0.70, g - 1.48 * s, x - hw * 0.70, g - 1.48 * s, darker(body, 0.85));
    cv.quad(x - hw * 0.80, g - 1.02 * s, x + hw * 0.80, g - 1.02 * s,                              // rear window
            x + hw * 0.63, g - 1.42 * s, x - hw * 0.63, g - 1.42 * s, paint({45, 60, 78}));
    cv.rect(x - hw, g - 0.44 * s, x + hw, g - 0.30 * s, darker(body, 0.55));                       // bumper
    cv.rect(x - hw + 0.04 * s, g - 0.86 * s, x - hw + 0.38 * s, g - 0.68 * s, paint(kTailLight));
    cv.rect(x + hw - 0.38 * s, g - 0.86 * s, x + hw - 0.04 * s, g - 0.68 * s, paint(kTailLight));
    cv.rect(x - 0.24 * s, g - 0.64 * s, x + 0.24 * s, g - 0.51 * s, paint(kWhite));                // plate
    return {x - hw, g - 1.48 * s, x + hw, g};
}

Box drawTruck(Canvas& cv, int id, double x, double g, double s, Brush paint) {
    Colour cargo = paint(id % 2 ? Colour{238, 236, 228} : Colour{150, 70, 165});
    double hw = 1.3 * s;
    shadow(cv, x, g, hw * 1.2, s, paint.fog);
    cv.rect(x - hw + 0.05 * s, g - 0.55 * s, x - hw + 0.62 * s, g, paint(kTyre));                 // dual wheels
    cv.rect(x + hw - 0.62 * s, g - 0.55 * s, x + hw - 0.05 * s, g, paint(kTyre));
    cv.rect(x - hw * 0.9, g - 0.65 * s, x + hw * 0.9, g - 0.40 * s, paint({28, 28, 32}));          // chassis
    cv.rect(x - hw, g - 3.4 * s, x + hw, g - 0.65 * s, cargo);                                      // cargo box
    cv.rect(x, g - 3.4 * s, x + hw, g - 0.65 * s, darker(cargo, 0.93));
    cv.rect(x - 0.03 * s, g - 3.3 * s, x + 0.03 * s, g - 0.65 * s, darker(cargo, 0.6));            // doors
    cv.rect(x - 0.25 * s, g - 1.9 * s, x - 0.12 * s, g - 1.6 * s, darker(cargo, 0.5));
    cv.rect(x + 0.12 * s, g - 1.9 * s, x + 0.25 * s, g - 1.6 * s, darker(cargo, 0.5));
    for (int i = 0; i < 8; i++)                                                                      // reflective stripe
        cv.rect(x - hw + i * hw / 4, g - 0.82 * s, x - hw + (i + 1) * hw / 4, g - 0.72 * s,
                paint(i % 2 ? kWhite : Colour{220, 30, 30}));
    cv.rect(x - hw + 0.05 * s, g - 1.15 * s, x - hw + 0.35 * s, g - 0.9 * s, paint(kTailLight));
    cv.rect(x + hw - 0.35 * s, g - 1.15 * s, x + hw - 0.05 * s, g - 0.9 * s, paint(kTailLight));
    return {x - hw, g - 3.4 * s, x + hw, g};
}

Box drawPedestrian(Canvas& cv, int id, double x, double g, double s, Brush paint) {
    const Colour shirts[] = {{240, 190, 40}, {220, 70, 60}, {60, 125, 215}, {150, 85, 185}};
    Colour shirt = paint(shirts[id % 4]), trousers = paint({42, 52, 85}), skin = paint(kSkin);
    shadow(cv, x, g, 0.38 * s, s, paint.fog);
    cv.rect(x - 0.17 * s, g - 0.90 * s, x - 0.03 * s, g, trousers);                                // legs
    cv.rect(x + 0.03 * s, g - 0.90 * s, x + 0.17 * s, g, trousers);
    cv.rect(x - 0.19 * s, g - 0.06 * s, x - 0.02 * s, g, paint({28, 28, 32}));                     // shoes
    cv.rect(x + 0.02 * s, g - 0.06 * s, x + 0.19 * s, g, paint({28, 28, 32}));
    cv.rect(x - 0.31 * s, g - 1.45 * s, x - 0.22 * s, g - 0.92 * s, darker(shirt, 0.85));          // arms
    cv.rect(x + 0.22 * s, g - 1.45 * s, x + 0.31 * s, g - 0.92 * s, darker(shirt, 0.85));
    cv.ellipse(x - 0.265 * s, g - 0.90 * s, 0.05 * s, 0.05 * s, skin);                             // hands
    cv.ellipse(x + 0.265 * s, g - 0.90 * s, 0.05 * s, 0.05 * s, skin);
    cv.rect(x - 0.23 * s, g - 1.50 * s, x + 0.23 * s, g - 0.86 * s, shirt);                        // body
    cv.rect(x - 0.05 * s, g - 1.56 * s, x + 0.05 * s, g - 1.48 * s, skin);                         // neck
    cv.ellipse(x, g - 1.66 * s, 0.12 * s, 0.14 * s, skin);                                          // head
    cv.ellipse(x, g - 1.70 * s, 0.125 * s, 0.11 * s, paint({48, 34, 26}));                         // hair
    return {x - 0.32 * s, g - 1.80 * s, x + 0.32 * s, g};
}

Box drawCyclist(Canvas& cv, int id, double x, double g, double s, Brush paint) {
    Colour frame = paint({215, 55, 55}), jersey = paint(id % 2 ? Colour{30, 150, 210} : Colour{250, 200, 30});
    double wheelR = 0.33 * s, rearX = x - 0.55 * s, frontX = x + 0.55 * s, hubY = g - wheelR;
    double seatX = x - 0.15 * s, seatY = g - 0.95 * s, barX = x + 0.42 * s, barY = g - 1.02 * s;
    shadow(cv, x, g, 0.95 * s, s, paint.fog);
    cv.ring(rearX, hubY, wheelR, 0.06 * s, paint(kTyre));                                           // wheels
    cv.ring(frontX, hubY, wheelR, 0.06 * s, paint(kTyre));
    cv.line(rearX, hubY, seatX, seatY, 0.05 * s, frame);                                            // bike frame
    cv.line(rearX, hubY, x, hubY, 0.05 * s, frame);
    cv.line(x, hubY, seatX, seatY, 0.05 * s, frame);
    cv.line(x, hubY, barX, barY, 0.05 * s, frame);
    cv.line(barX, barY, frontX, hubY, 0.05 * s, frame);
    cv.line(seatX, seatY, x + 0.05 * s, g - 0.45 * s, 0.13 * s, paint({35, 35, 40}));              // leg
    cv.line(seatX, seatY - 0.05 * s, x + 0.18 * s, g - 1.42 * s, 0.24 * s, jersey);                // body
    cv.line(x + 0.18 * s, g - 1.38 * s, barX, barY, 0.08 * s, jersey);                              // arm
    cv.ellipse(x + 0.30 * s, g - 1.60 * s, 0.12 * s, 0.13 * s, paint(kSkin));                       // head
    cv.ellipse(x + 0.28 * s, g - 1.67 * s, 0.15 * s, 0.09 * s, paint({245, 245, 245}));            // helmet
    return {x - 0.9 * s, g - 1.78 * s, x + 0.9 * s, g};
}

Box drawCone(Canvas& cv, double x, double g, double s, Brush paint) {
    shadow(cv, x, g, 0.36 * s, s, paint.fog);
    cv.rect(x - 0.30 * s, g - 0.07 * s, x + 0.30 * s, g, paint({200, 85, 10}));                    // base
    cv.triangle(x - 0.23 * s, g - 0.07 * s, x + 0.23 * s, g - 0.07 * s, x, g - 0.80 * s, paint({255, 120, 15}));
    auto halfWidthAt = [](double height) { return 0.23 * (1 - (height - 0.07) / 0.73); };
    for (auto band : {std::array<double, 2>{0.28, 0.40}, {0.50, 0.59}}) {                          // white bands
        double hw = (halfWidthAt(band[0]) + halfWidthAt(band[1])) / 2;
        cv.rect(x - hw * s, g - band[1] * s, x + hw * s, g - band[0] * s, paint(kWhite));
    }
    return {x - 0.30 * s, g - 0.80 * s, x + 0.30 * s, g};
}

Box drawSign(Canvas& cv, double x, double g, double s, Brush paint) {   // "no entry" sign on a pole
    shadow(cv, x, g, 0.2 * s, s, paint.fog);
    cv.rect(x - 0.04 * s, g - 1.9 * s, x + 0.04 * s, g, paint({150, 150, 158}));
    cv.ellipse(x, g - 2.1 * s, 0.44 * s, 0.44 * s, paint(kWhite));
    cv.ellipse(x, g - 2.1 * s, 0.39 * s, 0.39 * s, paint({205, 30, 35}));
    cv.rect(x - 0.26 * s, g - 2.16 * s, x + 0.26 * s, g - 2.04 * s, paint(kWhite));
    return {x - 0.44 * s, g - 2.54 * s, x + 0.44 * s, g};
}

Box drawObstacle(Canvas& cv, const world::Obstacle& o, double x, double g, double s, Brush paint) {
    if (o.kind == "car") return drawCar(cv, o.id, x, g, s, paint);
    if (o.kind == "truck") return drawTruck(cv, o.id, x, g, s, paint);
    if (o.kind == "pedestrian") return drawPedestrian(cv, o.id, x, g, s, paint);
    if (o.kind == "cyclist") return drawCyclist(cv, o.id, x, g, s, paint);
    if (o.kind == "cone") return drawCone(cv, x, g, s, paint);
    return drawSign(cv, x, g, s, paint);
}

// ------------------------------------------------------------------------------------------------
// BMP file
// ------------------------------------------------------------------------------------------------
template <class T>
void append(std::vector<uint8_t>& out, T value) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&value);
    out.insert(out.end(), p, p + sizeof(T));
}

std::vector<uint8_t> toBmp(const Canvas& cv) {
    const uint32_t pixelBytes = W * H * 3;
    std::vector<uint8_t> bmp;
    bmp.reserve(54 + pixelBytes);
    bmp.push_back('B');
    bmp.push_back('M');
    append<uint32_t>(bmp, 54 + pixelBytes);   // file size
    append<uint32_t>(bmp, 0);
    append<uint32_t>(bmp, 54);                // where the pixels start
    append<uint32_t>(bmp, 40);                // info header size
    append<int32_t>(bmp, W);
    append<int32_t>(bmp, H);
    append<uint16_t>(bmp, 1);                 // planes
    append<uint16_t>(bmp, 24);                // bits per pixel
    append<uint32_t>(bmp, 0);                 // no compression
    append<uint32_t>(bmp, pixelBytes);
    append<int32_t>(bmp, 2835);
    append<int32_t>(bmp, 2835);
    append<uint32_t>(bmp, 0);
    append<uint32_t>(bmp, 0);
    for (int y = H - 1; y >= 0; y--)          // BMP stores rows bottom-up, pixels as blue-green-red
        for (int x = 0; x < W; x++) {
            const uint8_t* p = &cv.rgb[((size_t)y * W + x) * 3];
            bmp.push_back(p[2]);
            bmp.push_back(p[1]);
            bmp.push_back(p[0]);
        }
    return bmp;
}

}  // namespace

RenderedImage renderDashcam(const world::World& world, const world::Pose& car, int focusObstacleId) {
    Canvas cv;
    drawSky(cv, car.heading * kFocal);   // distant things only move when the car turns
    drawGround(cv, car);

    // Collect the obstacles in view and draw them far to near, so near ones cover far ones.
    struct InView {
        const world::Obstacle* obstacle;
        double ahead, left;   // metres, relative to the camera
    };
    std::vector<InView> inView;
    const double cosH = std::cos(car.heading), sinH = std::sin(car.heading);
    for (const auto& o : world.obstacles()) {
        double dx, dy;
        world.offsetTo(o, car.x, car.y, dx, dy);
        double ahead = dx * cosH + dy * sinH, left = -dx * sinH + dy * cosH;
        if (ahead < 2.0 || ahead > 130.0) continue;
        if (std::fabs(left / ahead) > std::tan(kHalfFov) + 0.4) continue;   // outside the picture
        inView.push_back({&o, ahead, left});
    }
    std::sort(inView.begin(), inView.end(), [](const InView& a, const InView& b) { return a.ahead > b.ahead; });

    RenderedImage result;
    for (const auto& v : inView) {
        double s = kFocal / v.ahead;   // pixels per metre at this distance
        double x = W / 2.0 - v.left * s, ground = kHorizon + kCameraHeight * s;
        Box b = drawObstacle(cv, *v.obstacle, x, ground, s, Brush{fogAt(v.ahead)});
        if (v.obstacle->id == focusObstacleId) {
            result.hasBox = true;
            result.box[0] = (int)std::clamp(b.x0 - 4, 0.0, W - 1.0);
            result.box[1] = (int)std::clamp(b.y0 - 4, 0.0, H - 1.0);
            result.box[2] = (int)std::clamp(b.x1 + 4, 0.0, W - 1.0);
            result.box[3] = (int)std::clamp(b.y1 + 3, 0.0, H - 1.0);
        }
    }
    drawBonnet(cv);
    result.bmp = toBmp(cv);
    return result;
}

}  // namespace sensors
