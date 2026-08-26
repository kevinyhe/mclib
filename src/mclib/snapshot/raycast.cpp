// mclib
#include "mclib/snapshot/raycast.hpp"
#include <algorithm>
#include <cmath>

namespace snapshot
{

  float cross(const Vec2 &a, const Vec2 &b) { return a.x * b.y - a.y * b.x; }
  Vec2 sub(const Vec2 &a, const Vec2 &b) { return Vec2{a.x - b.x, a.y - b.y}; }
  Vec2 add(const Vec2 &a, const Vec2 &b) { return Vec2{a.x + b.x, a.y + b.y}; }
  Vec2 mul(const Vec2 &a, float s) { return Vec2{a.x * s, a.y * s}; }
  float dot(const Vec2 &a, const Vec2 &b) { return a.x * b.x + a.y * b.y; }
  float norm(const Vec2 &a) { return std::sqrt(dot(a, a)); }

  Vec2 unit_from_heading_deg(float heading_deg)
  {
    // 0 deg is +Y, clockwise-positive
    // (sin(theta), cos(theta))
    const float th = heading_deg * 3.1415926535f / 180.0f;
    return Vec2{std::sin(th), std::cos(th)};
  }

  static bool ray_segment_intersect(
      const Vec2 &p, const Vec2 &r_unit,
      const Vec2 &a, const Vec2 &b,
      float &out_t, float &out_u)
  {
    const Vec2 q = a;
    const Vec2 s = sub(b, a);

    const float rxs = cross(r_unit, s);
    const float q_pxs = cross(sub(q, p), s);

    const float eps = 1e-8f;
    if (std::fabs(rxs) < eps)
      return false; // parallel

    const float t = q_pxs / rxs;
    const float u = cross(sub(q, p), r_unit) / rxs;

    if (t < 0.0f)
      return false;
    if (u < 0.0f || u > 1.0f)
      return false;

    out_t = t;
    out_u = u;
    return true;
  }

  std::vector<RayHit> raycast_all(
      const Vec2 &origin,
      const Vec2 &dir_unit,
      float max_range_in,
      std::uint32_t allow_mask)
  {
    std::vector<RayHit> hits;
    hits.reserve(16);

    for (std::size_t i = 0; i < TERMINAL_FIELD_SEGMENTS.size(); ++i)
    {
      const auto &seg = TERMINAL_FIELD_SEGMENTS[i];
      if ((seg.mask & allow_mask) == 0u)
        continue;

      float t = 0.0f, u = 0.0f;
      if (!ray_segment_intersect(origin, dir_unit, seg.a, seg.b, t, u))
        continue;
      if (t > max_range_in)
        continue;

      const Vec2 pt = add(origin, mul(dir_unit, t));
      hits.push_back(RayHit{t, i, pt});
    }

    std::sort(hits.begin(), hits.end(), [](const RayHit &h1, const RayHit &h2)
              { return h1.t_in < h2.t_in; });

    return hits;
  }

} // namespace snapshot
