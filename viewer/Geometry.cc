#include <Geometry.h>
#include <Sim/World.h>

namespace viewer {
namespace {

using Color = std::array<float, 3>;

// Colors from the old fixed-function Draw() code.
constexpr Color kWhite{1, 1, 1};
constexpr Color kBlack{0, 0, 0};
constexpr Color kRed{1, 0, 0};
constexpr Color kGreen{0, 1, 0};
constexpr Color kBlue{0, 0, 1};
constexpr Color kSurfaceGray{0.7f, 0.7f, 0.7f};
constexpr Color kGridGray{0.5f, 0.5f, 0.5f};

constexpr float kVertexPointSize = 4;
constexpr float kPinnedPointSize = 5;
constexpr float kContactPointSize = 7;

auto ToFloat(const Vec3<Real> &v) -> std::array<float, 3> {
  return {static_cast<float>(v.x()), static_cast<float>(v.y()),
          static_cast<float>(v.z())};
}

void Line(FrameGeometry &g, const Vec3<Real> &a, const Vec3<Real> &b,
          Color color) {
  g.lines.push_back({ToFloat(a), color, 1});
  g.lines.push_back({ToFloat(b), color, 1});
}

void Point(FrameGeometry &g, const Vec3<Real> &p, Color color, float size) {
  g.points.push_back({ToFloat(p), color, size});
}

// Lit flat-shaded surface plus a black wireframe, the old DrawSurfaceMesh.
void Surface(FrameGeometry &g, const Mat<Real> &v, const Mat<int> &f) {
  for (int ii = 0; ii < f.rows(); ++ii) {
    const Vec3<Real> a = v.row(f(ii, 0));
    const Vec3<Real> b = v.row(f(ii, 1));
    const Vec3<Real> c = v.row(f(ii, 2));
    const auto normal = ToFloat((b - a).cross(c - a).normalized());
    g.triangles.push_back({ToFloat(a), normal, kSurfaceGray});
    g.triangles.push_back({ToFloat(b), normal, kSurfaceGray});
    g.triangles.push_back({ToFloat(c), normal, kSurfaceGray});
    Line(g, a, b, kBlack);
    Line(g, b, c, kBlack);
    Line(g, c, a, kBlack);
  }
}

void AppendStrand(FrameGeometry &g, const StrandMesh &strand,
                  bool materialFrames) {
  const auto &der = *strand.der;
  for (int ii = 0; ii < der.nRods; ++ii) {
    const Vec3<Real> x0 = der.vertices.row(ii);
    const Vec3<Real> x1 = der.vertices.row(ii + 1);
    Line(g, x0, x1, kWhite);
    if (materialFrames) {
      const auto &frame = der.frames.at(ii);
      Line(g, x0, x0 + frame.t, kRed);
      Line(g, x0, x0 + frame.u, kGreen);
      Line(g, x0, x0 + frame.v, kBlue);
    }
  }
  for (int ii = 0; ii < der.vertices.rows(); ++ii) {
    Point(g, der.vertices.row(ii), kWhite, kVertexPointSize);
  }
}

void AppendSoftBody(FrameGeometry &g, const TetMesh &mesh) {
  Surface(g, mesh.v, mesh.f);
  for (int ii = 0; ii < mesh.pinned.size(); ++ii) {
    if (mesh.pinned(ii) == 1) {
      Point(g, mesh.v.row(ii), kRed, kPinnedPointSize);
    }
  }
}

} // namespace

void FrameGeometry::Clear() {
  lines.clear();
  points.clear();
  triangles.clear();
}

void AppendWorld(FrameGeometry &geometry, const sim::World &world) {
  const bool frames = world.Desc().view.showMaterialFrames;
  for (const auto &body : world.Bodies()) {
    std::visit(
        [&](const auto *mesh) {
          using T = std::remove_cvref_t<decltype(*mesh)>;
          if constexpr (std::is_same_v<T, StrandMesh>) {
            AppendStrand(geometry, *mesh, frames);
          } else if constexpr (std::is_same_v<T, TetMesh>) {
            AppendSoftBody(geometry, *mesh);
          } else {
            Surface(geometry, mesh->v, mesh->f);
          }
        },
        body);
  }
  for (const auto &contact : world.Contacts()) {
    for (const auto &corner : contact.faceVertices) {
      Point(geometry, corner, kRed, kContactPointSize);
    }
  }
}

void AppendGridAndAxes(FrameGeometry &geometry) {
  constexpr int kSize = 100;
  constexpr float kSpacing = 0.25f;
  for (float ii = -kSize; ii < kSize; ii += kSpacing) {
    Line(geometry, {ii, 0, -kSize}, {ii, 0, kSize}, kGridGray);
    Line(geometry, {kSize, 0, ii}, {-kSize, 0, ii}, kGridGray);
  }
  const Vec3<Real> origin = Vec3<Real>::Zero();
  Line(geometry, origin, Vec3<Real>::UnitX(), kRed);
  Line(geometry, origin, Vec3<Real>::UnitY(), kGreen);
  Line(geometry, origin, Vec3<Real>::UnitZ(), kBlue);
}

} // namespace viewer
