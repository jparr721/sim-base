#pragma once

#include "LibMath.h"

template <typename Real> class Camera {
public:
  Camera();
  virtual ~Camera();

  auto Resize(std::size_t width, std::size_t height) noexcept -> bool;
  auto Resize(std::size_t width, std::size_t height, Real nearPlane,
              Real farPlane) noexcept -> bool;

  auto Rotate(Real du, Real dv) -> bool;
  auto Pan(Real du, Real dv) -> bool;
  auto Zoom(Real du) -> bool;

  void SetPerspective(Real fov, std::size_t width, std::size_t height,
                      Real nearPlane, Real farPlane);
  void SetFrustum(Real left, Real right, Real top, Real bottom, Real nearPlane,
                  Real farPlane);

  void SetPosition(const Vec3<Real> &position);
  void SetPosition(Real x, Real y, Real z);
  void AddPosition(const Vec3<Real> &displacement);
  void AddPosition(Real dx, Real dy, Real dz);

  void SetRotation(Real theta, Real phi);
  void SetHorizontalRotation(Real theta);
  void SetVerticalRotation(Real phi);
  void SetSphericalPosition(Real r, Real theta, Real phi);
  void AddRadius(Real dRadius);
  void SetRadius(Real radius);
  void SetFieldOfView(Real fov);
  void SetNearPlane(Real nearPlane);
  void SetFarPlane(Real farPlane);
  void SetMinRadius(Real minRadius);
  void SetMaxRadius(Real maxRadius);

  void SetRotationSensitivity(Real sen);
  void SetPanSensitivity(Real sen);
  void SetZoomSensitivity(Real sen);
  void SetScopeSensitivity(Real sen);

  void Reset();
  void ResetPlanes();
  void ResetView();
  void ResetMatrices();
  void ResetSensitivities();

  auto GetRotationSensitivity() const -> Real;
  auto GetPanSensitivity() const -> Real;
  auto GetZoomSensitivity() const -> Real;
  auto GetScopeSensitivity() const -> Real;

  auto GetViewDirection() -> Vec3<Real>;
  auto GetRightDirection() -> Vec3<Real>;
  auto GetLeftDirection() -> Vec3<Real>;
  auto GetUpDirection() -> Vec3<Real>;
  auto GetDownDirection() -> Vec3<Real>;

  auto GetEye() const -> const Vec3<Real> &;
  auto GetLookAt() const -> const Vec3<Real> &;
  auto GetUp() const -> const Vec3<Real> &;

  std::size_t GetWidth() const;
  auto GetHeight() const -> std::size_t;
  auto GetNear() const -> const Real &;
  auto GetFar() const -> const Real &;

  auto GetViewMatrix() -> const Matrix4<Real> &;
  auto GetProjectionMatrix() -> const Matrix4<Real> &;

  auto ToViewMatrix() -> Matrix4<Real>;
  auto ToProjectionMatrix() -> Matrix4<Real>;

  static auto LookAt(const Vec3<Real> &eye, const Vec3<Real> &lookAt,
                     const Vec3<Real> &up) -> Matrix4<Real>;
  static auto LookAt(Real eyex, Real eyey, Real eyez, Real atx, Real aty,
                     Real atz, Real upx, Real upy, Real upz) -> Matrix4<Real>;

  static auto PerspectiveMatrix(Real fov, std::size_t width, std::size_t height,
                                Real nearPlane, Real farPlane) noexcept
      -> Matrix4<Real>;
  static auto OrthographicMatrix(Real left, Real right, Real bottom, Real top,
                                 Real nearPlane, Real farPlane) noexcept
      -> Matrix4<Real>;

  static auto SphereicalToCartesian(Real r, Real theta, Real phi)
      -> Vec3<Real>;
  static auto SphereicalToCartesian_dTheta(Real r, Real theta, Real phi)
      -> Vec3<Real>;
  static auto SphereicalToCartesian_dPhi(Real r, Real theta, Real phi)
      -> Vec3<Real>;
  static auto SphereicalToCartesian_dPhiCrossdTheta(Real r, Real theta,
                                                    Real phi) -> Vec3<Real>;

protected:
  void compile() noexcept;

protected:
  Matrix4<Real> viewMatrix;
  Matrix4<Real> projectionMatrix;
  Real nearPlane, farPlane;

  std::size_t width, height;
  Vec3<Real> eye;
  Vec3<Real> lookAt;
  Vec3<Real> up;

  Real fov, aspectRatio;
  Real minRadius, maxRadius;

  Real r, theta, phi;
  Vec3<Real> displacement;

  Real panSensitivity;
  Real zoomSensitivity;
  Real rotationSensitivity;
  Real scopeSensitivity;
};

const Real CAMERA_EPSILON = Real(0.00001);
const Real DEFAULT_NEAR_PLANE = Real(0.1);
const Real DEFAULT_FAR_PLANE = Real(1000.0);
const Real DEFAULT_ROTATION_SENSITIVITY = Real(0.01);
const Real DEFAULT_PAN_SENSITIVITY = Real(0.01);
const Real DEFAULT_ZOOM_SENSITIVITY = Real(0.1);
const Real DEFAULT_SCOPE_SENSITIVITY = Real(0.01);
const Real DEFAULT_RADIUS = Real(1);
const Real DEFAULT_THETA = Real(1.57079632679);
const Real DEFAULT_PHI = Real(1.57079632679);
const Real DEFAULT_FOV = Real(65.0);
const Real DEFAULT_ASPECT_RATIO = Real(1.0);
const Real DEFAULT_MIN_RADIUS = Real(0.1);
const Real DEFAULT_MAX_RADIUS = Real(1000.0);
const Real MAX_FOV = Real(120.0);
const Real MIN_FOV = Real(10.0);

template <typename Real> Camera<Real>::Camera() {
  this->viewMatrix = Matrix4<Real>::Identity();
  this->projectionMatrix = Matrix4<Real>::Identity();

  this->eye = Vec3<Real>::UnitZ();
  this->lookAt = Vec3<Real>::Zero();
  this->up = Vec3<Real>::UnitY();

  this->nearPlane = Real(DEFAULT_NEAR_PLANE);
  this->farPlane = Real(DEFAULT_FAR_PLANE);

  this->rotationSensitivity = Real(DEFAULT_ROTATION_SENSITIVITY);
  this->panSensitivity = Real(DEFAULT_PAN_SENSITIVITY);
  this->zoomSensitivity = Real(DEFAULT_ZOOM_SENSITIVITY);
  this->scopeSensitivity = Real(DEFAULT_SCOPE_SENSITIVITY);

  this->r = Real(1);
  this->theta = Real(DEFAULT_THETA);
  this->phi = Real(DEFAULT_PHI);
  this->fov = Real(DEFAULT_FOV);
  this->aspectRatio = Real(DEFAULT_ASPECT_RATIO);
  this->minRadius = Real(DEFAULT_MIN_RADIUS);
  this->maxRadius = Real(DEFAULT_MAX_RADIUS);
  this->displacement = Vec3<Real>::Zero();
}

template <typename Real> Camera<Real>::~Camera() = default;

template <typename Real>
auto Camera<Real>::Resize(std::size_t width, std::size_t height) noexcept
    -> bool {
  this->SetPerspective(this->fov, width, height, this->nearPlane,
                       this->farPlane);
  this->compile();
  return true;
}

template <typename Real>
auto Camera<Real>::Resize(std::size_t width, std::size_t height, Real nearPlane,
                          Real farPlane) noexcept -> bool {
  this->SetPerspective(this->fov, width, height, this->nearPlane,
                       this->farPlane);
  this->compile();
  return true;
}

template <typename Real> auto Camera<Real>::Rotate(Real du, Real dv) -> bool {
  this->theta -= (du * this->rotationSensitivity);
  this->phi += (dv * this->rotationSensitivity);
  this->compile();
  return true;
}

template <typename Real> auto Camera<Real>::Pan(Real du, Real dv) -> bool {
  Vec3<Real> uDir = this->GetLeftDirection();
  Vec3<Real> vDir = this->GetDownDirection();

  Vec3<Real> uDisp = (du * this->panSensitivity) * uDir;
  Vec3<Real> vDisp = (dv * this->panSensitivity) * vDir;
  Vec3<Real> panDisp = uDisp + vDisp;

  this->displacement += panDisp;
  this->compile();
  return true;
}

template <typename Real> auto Camera<Real>::Zoom(Real dr) -> bool {
  if (this->r + dr > this->maxRadius) {
    this->r = this->maxRadius;
    this->compile();
    return false;
  } else if (this->r + dr < this->minRadius) {
    this->r = this->minRadius;
    this->compile();
    return false;
  } else {
    this->r += dr;
    this->compile();
    return true;
  }

  return false;
}

template <typename Real>
void Camera<Real>::SetPerspective(Real fov, std::size_t width,
                                  std::size_t height, Real nearPlane,
                                  Real farPlane) {
  if (fov > Real(MAX_FOV))
    this->fov = Real(MAX_FOV);
  else if (fov < Real(MIN_FOV))
    this->fov = Real(MIN_FOV);
  else
    this->fov = fov;

  this->width = width;
  this->height = height;
  this->nearPlane = nearPlane;
  this->farPlane = farPlane;

  Real aspectRatio = static_cast<Real>(width) / static_cast<Real>(height);
  Real ymax = nearPlane * std::tan(fov * Real(PI) / Real(360));
  Real xmax = ymax * aspectRatio;
  this->SetFrustum(-xmax, xmax, ymax, -ymax, nearPlane, farPlane);
}

template <typename Real>
void Camera<Real>::SetFrustum(Real left, Real right, Real top, Real bottom,
                              Real nearPlane, Real farPlane) {
  this->projectionMatrix.setZero();

  Real temp, temp2, temp3, temp4;
  temp = Real(2) * nearPlane;
  temp2 = right - left;
  temp3 = top - bottom;
  temp4 = farPlane - nearPlane;

  this->projectionMatrix(0) = temp / temp2;
  this->projectionMatrix(5) = temp / temp3;
  this->projectionMatrix(8) = (right + left) / temp2;
  this->projectionMatrix(9) = (top + bottom) / temp3;
  this->projectionMatrix(10) = (-farPlane - nearPlane) / temp4;
  this->projectionMatrix(11) = Real(-1);
  this->projectionMatrix(14) = (-temp * farPlane) / temp4;
}

template <typename Real>
void Camera<Real>::SetPosition(const Vec3<Real> &pos) {
  this->position.x() = pos.x();
  this->position.y() = pos.y();
  this->position.z() = pos.z();
  this->compile();
}

template <typename Real>
void Camera<Real>::SetPosition(Real x, Real y, Real z) {
  this->position.x() = x;
  this->position.y() = y;
  this->position.z() = z;
  this->compile();
}

template <typename Real>
void Camera<Real>::AddPosition(const Vec3<Real> &displacement) {
  this->position += displacement;
  this->compile();
}

template <typename Real>
void Camera<Real>::AddPosition(Real dx, Real dy, Real dz) {
  this->displacement.x() += dx;
  this->displacement.y() += dy;
  this->displacement.z() += dz;
  this->compile();
}

template <typename Real> void Camera<Real>::SetRotation(Real theta, Real phi) {
  this->theta = theta;
  this->phi = phi;
  this->compile();
}

template <typename Real> void Camera<Real>::SetHorizontalRotation(Real theta) {
  this->theta = theta;
  this->compile();
}

template <typename Real> void Camera<Real>::SetVerticalRotation(Real phi) {
  this->phi = phi;
  this->compile();
}

template <typename Real>
void Camera<Real>::SetSphericalPosition(Real r, Real theta, Real phi) {
  this->r = r;
  this->theta = theta;
  this->phi = phi;
  this->compile();
}

template <typename Real> void Camera<Real>::AddRadius(Real dRadius) {
  this->r += dRadius;
  this->compile();
}

template <typename Real> void Camera<Real>::SetRadius(Real radius) {
  this->r = radius;
  this->compile();
}

template <typename Real> void Camera<Real>::SetFieldOfView(Real fov) {
  if (fov > Real(MAX_FOV))
    this->fov = Real(MAX_FOV);
  else if (fov < Real(MIN_FOV))
    this->fov = Real(MIN_FOV);
  else
    this->fov = fov;
  this->SetPerspective(this->fov, this->aspectRatio, this->nearPlane,
                       this->farPlane);
}

template <typename Real> void Camera<Real>::SetNearPlane(Real nearPlane) {
  this->nearPlane = nearPlane;
  this->SetPerspective(this->fov, this->aspectRatio, this->nearPlane,
                       this->farPlane);
}

template <typename Real> void Camera<Real>::SetFarPlane(Real farPlane) {
  this->farPlane = farPlane;
  this->SetPerspective(this->fov, this->aspectRatio, this->nearPlane,
                       this->farPlane);
}

template <typename Real> void Camera<Real>::SetMinRadius(Real minRadius) {
  this->minRadius = minRadius;
}

template <typename Real> void Camera<Real>::SetMaxRadius(Real maxRadius) {
  this->maxRadius = maxRadius;
}

template <typename Real> void Camera<Real>::SetRotationSensitivity(Real sen) {
  this->rotationSensitivity = sen;
}

template <typename Real> void Camera<Real>::SetPanSensitivity(Real sen) {
  this->panSensitivity = sen;
}

template <typename Real> void Camera<Real>::SetZoomSensitivity(Real sen) {
  this->zoomSensitivity = sen;
}

template <typename Real> void Camera<Real>::SetScopeSensitivity(Real sen) {
  this->scopeSensitivity = sen;
}

template <typename Real> void Camera<Real>::Reset() {
  this->ResetMatrices();
  this->ResetPlanes();
  this->ResetView();
  this->ResetSensitivities();
  this->compile();
}

template <typename Real> void Camera<Real>::ResetPlanes() {
  this->nearPlane = Real(DEFAULT_NEAR_PLANE);
  this->farPlane = Real(DEFAULT_FAR_PLANE);
}

template <typename Real> void Camera<Real>::ResetView() {
  this->eye = Vec3<Real>::UnitZ();
  this->lookAt = Vec3<Real>::Zero();
  this->up = Vec3<Real>::UnitY();
}

template <typename Real> void Camera<Real>::ResetMatrices() {
  this->viewMatrix = Matrix4<Real>::Identity();
  this->projectionMatrix = Matrix4<Real>::Identity();
}

template <typename Real> void Camera<Real>::ResetSensitivities() {
  this->rotationSensitivity = Real(DEFAULT_ROTATION_SENSITIVITY);
  this->panSensitivity = Real(DEFAULT_PAN_SENSITIVITY);
  this->zoomSensitivity = Real(DEFAULT_ZOOM_SENSITIVITY);
  this->scopeSensitivity = Real(DEFAULT_SCOPE_SENSITIVITY);
}

template <typename Real>
auto Camera<Real>::GetRotationSensitivity() const -> Real {
  return this->rotationSensitivity;
}

template <typename Real> auto Camera<Real>::GetPanSensitivity() const -> Real {
  return this->panSensitivity;
}

template <typename Real> auto Camera<Real>::GetZoomSensitivity() const -> Real {
  return this->zoomSensitivity;
}

template <typename Real>
auto Camera<Real>::GetScopeSensitivity() const -> Real {
  return this->scopeSensitivity;
}

template <typename Real>
inline auto Camera<Real>::GetViewDirection() -> Vec3<Real> {
  this->compile();
  return (this->lookAt - this->eye).normalized();
}

template <typename Real>
inline auto Camera<Real>::GetRightDirection() -> Vec3<Real> {
  this->compile();
  Vec3<Real> dir = (this->lookAt - this->eye).normalized();
  return (this->up.cross(dir)).normalized();
}

template <typename Real>
inline auto Camera<Real>::GetLeftDirection() -> Vec3<Real> {
  this->compile();
  Vec3<Real> dir = (this->lookAt - this->eye).normalized();
  return (dir.cross(this->up)).normalized();
}

template <typename Real>
inline auto Camera<Real>::GetUpDirection() -> Vec3<Real> {
  this->compile();
  return this->up;
}

template <typename Real>
inline auto Camera<Real>::GetDownDirection() -> Vec3<Real> {
  this->compile();
  return -this->up;
}

template <typename Real>
auto Camera<Real>::GetEye() const -> const Vec3<Real> & {
  return this->eye;
}

template <typename Real>
auto Camera<Real>::GetLookAt() const -> const Vec3<Real> & {
  return this->lookAt;
}

template <typename Real>
auto Camera<Real>::GetUp() const -> const Vec3<Real> & {
  return this->up;
}

template <typename Real> auto Camera<Real>::GetWidth() const -> std::size_t {
  return this->width;
}

template <typename Real> auto Camera<Real>::GetHeight() const -> std::size_t {
  return this->height;
}

template <typename Real> auto Camera<Real>::GetNear() const -> const Real & {
  return this->nearPlane;
}

template <typename Real> auto Camera<Real>::GetFar() const -> const Real & {
  return this->farPlane;
}

template <typename Real>
auto Camera<Real>::GetViewMatrix() -> const Matrix4<Real> & {
  this->compile();
  return this->viewMatrix;
}

template <typename Real>
auto Camera<Real>::GetProjectionMatrix() -> const Matrix4<Real> & {
  this->compile();
  return this->projectionMatrix;
}

template <typename Real> auto Camera<Real>::ToViewMatrix() -> Matrix4<Real> {
  this->compile();
  return this->viewMatrix;
}

template <typename Real>
auto Camera<Real>::ToProjectionMatrix() -> Matrix4<Real> {
  this->compile();
  return this->projectionMatrix;
}

template <typename Real>
auto Camera<Real>::LookAt(const Vec3<Real> &eye, const Vec3<Real> &lookAt,
                          const Vec3<Real> &up) -> Matrix4<Real> {
  return Camera<Real>::LookAt(eye.x(), eye.y(), eye.z(), lookAt.x(), lookAt.y(),
                              lookAt.z(), up.x(), up.y(), up.z());
}

template <typename Real>
auto Camera<Real>::LookAt(Real eyex, Real eyey, Real eyez, Real atx, Real aty,
                          Real atz, Real upx, Real upy, Real upz)
    -> Matrix4<Real> {
  Matrix4<Real> matrix;
  Vec3<Real> x, y, z;
  Vec3<Real> eye(eyex, eyey, eyez);

  y = Vec3<Real>(upx, upy, upz);
  z = Vec3<Real>(atx - eyex, aty - eyey, atz - eyez);
  x = y.cross(z).normalized();
  y = z.cross(x).normalized();
  z.normalize();

  matrix(0, 0) = -x.x();
  matrix(0, 1) = -x.y();
  matrix(0, 2) = -x.z();
  matrix(0, 3) = x.dot(eye);

  matrix(1, 0) = y.x();
  matrix(1, 1) = y.y();
  matrix(1, 2) = y.z();
  matrix(1, 3) = -y.dot(eye);

  matrix(2, 0) = -z.x();
  matrix(2, 1) = -z.y();
  matrix(2, 2) = -z.z();
  matrix(2, 3) = z.dot(eye);

  matrix(3, 0) = Real(0);
  matrix(3, 1) = Real(0);
  matrix(3, 2) = Real(0);
  matrix(3, 3) = Real(1);

  return matrix;
}

template <typename Real>
auto Camera<Real>::PerspectiveMatrix(Real fov, std::size_t width,
                                     std::size_t height, Real nearPlane,
                                     Real farPlane) noexcept -> Matrix4<Real> {
  Matrix4<Real> proj;
  proj.setZero();

  Real aspectRatio = static_cast<Real>(width) / static_cast<Real>(height);
  Real ymax = nearPlane * std::tan(fov * Real(PI) / Real(360));
  Real xmax = ymax * aspectRatio;
  Real left = -xmax;
  Real right = xmax;
  Real top = ymax;
  Real bottom = -ymax;

  Real temp, temp2, temp3, temp4;
  temp = Real(2) * nearPlane;
  temp2 = right - left;
  temp3 = top - bottom;
  temp4 = farPlane - nearPlane;

  proj(0) = temp / temp2;
  proj(5) = temp / temp3;
  proj(8) = (right + left) / temp2;
  proj(9) = (top + bottom) / temp3;
  proj(10) = (-farPlane - nearPlane) / temp4;
  proj(11) = Real(-1);
  proj(14) = (-temp * farPlane) / temp4;

  return proj;
}

template <typename Real>
auto Camera<Real>::OrthographicMatrix(Real left, Real right, Real bottom,
                                      Real top, Real nearPlane,
                                      Real farPlane) noexcept -> Matrix4<Real> {
  Matrix4<Real> proj;

  Real x = Real(2) / (right - left);
  Real y = Real(2) / (top - bottom);
  Real z = Real(-2) / (farPlane - nearPlane);
  Real tx = (right + left) / (right - left);
  Real ty = (top + bottom) / (top - bottom);
  Real tz = (farPlane + nearPlane) / (farPlane - nearPlane);

  proj.setZero();

  proj(0, 0) = x;
  proj(1, 1) = y;
  proj(2, 2) = z;
  proj(3, 3) = Real(1);

  proj(0, 3) = -tx;
  proj(1, 3) = -ty;
  proj(2, 3) = -tz;

  return proj;
}

/*
 * Cartesian from Spherical (form)
 * x = r * cos(theta) * sin(phi)
 * y = r * sin(theta) * sin(phi)
 * z = r * cos(phi)
 *
 * Spherical from Cartesian (form)
 * r = sqrt(x^2 + y^2 + z^2)
 * theta = atan(y / x)
 * phi = acos(z / r);
 *
 * Unswapped: R(r, t, p) = rcos(theta)sin(phi)i + rsin(phi)sin(theta)j +
 * rcos(phi)k
 */
template <typename Real>
inline auto Camera<Real>::SphereicalToCartesian(Real r, Real theta, Real phi)
    -> Vec3<Real> {
  Vec3<Real> result;

  Real sinPhi = std::sin(phi);
  Real cosPhi = std::cos(phi);
  Real sinTheta = std::sin(theta);
  Real cosTheta = std::cos(theta);

  result.x() = r * (cosTheta * sinPhi);
  result.z() = r * (sinTheta * sinPhi);
  result.y() = r * cosPhi;
  return result;
}

/* Unswapped: Rt(r, t, p) = -rsin(phi)sin(theta)i + rsin(phi)cos(theta)j + 0k */
template <typename Real>
inline auto Camera<Real>::SphereicalToCartesian_dTheta(Real r, Real theta,
                                                       Real phi)
    -> Vec3<Real> {
  Vec3<Real> result;

  Real sinPhi = std::sin(phi);
  Real cosPhi = std::cos(phi);
  Real sinTheta = std::sin(theta);
  Real cosTheta = std::cos(theta);

  result.x() = -r * (sinPhi * sinTheta);
  result.z() = r * (sinPhi * cosTheta);
  result.y() = Real(0);
  return result;
}

/* Unswapped: Rp(r, t, p) = rcos(phi)cos(theta)i + rcos(phi)sin(theta)j -
 * rsin(phi)k */
template <typename Real>
inline auto Camera<Real>::SphereicalToCartesian_dPhi(Real r, Real theta,
                                                     Real phi)
    -> Vec3<Real> {
  Vec3<Real> result;

  Real sinPhi = std::sin(phi);
  Real cosPhi = std::cos(phi);
  Real sinTheta = std::sin(theta);
  Real cosTheta = std::cos(theta);

  result.x() = r * (cosPhi * cosTheta);
  result.z() = r * (cosPhi * sinTheta);
  result.y() = -r * sinPhi;
  return result;
}

/* Rp X Rt = r^2 * sin^2(phi)cos(theta)i + r^2 * sin^2(phi)sin(theta)j + r^2 *
 * sin(phi)cos(phi)k */
template <typename Real>
inline auto Camera<Real>::SphereicalToCartesian_dPhiCrossdTheta(Real r,
                                                                Real theta,
                                                                Real phi)
    -> Vec3<Real> {
  Vec3<Real> result;

  Real rs = (r * r);
  Real sinPhi = std::sin(phi);
  Real cosPhi = std::cos(phi);
  Real sinTheta = std::sin(theta);
  Real cosTheta = std::cos(theta);

  result.x() = -rs * ((sinPhi * sinPhi) * cosTheta);
  result.y() = -rs * ((sinPhi * sinPhi) * sinTheta);
  result.z() = -rs * sinPhi * cosPhi;
  return result;
}

template <typename Real> inline void Camera<Real>::compile() noexcept {
  this->lookAt = Vec3<Real>::Zero();
  this->eye =
      Camera<Real>::SphereicalToCartesian(this->r, this->theta, this->phi);
  this->up =
      Camera<Real>::SphereicalToCartesian_dPhi(this->r, this->theta, this->phi)
          .normalized();

  //--------------------------------------------------------------------------------
  // Invert the up direction (since the spherical coordinates have phi
  // increasing downwards. Therefore we would like to have the (vector)
  // direction of the derivative inversed.
  //--------------------------------------------------------------------------------
  this->up *= Real(-1.0);

  this->lookAt += this->displacement;
  this->eye += this->displacement;

  this->viewMatrix = Camera<Real>::LookAt(this->eye, this->lookAt, this->up);
}
