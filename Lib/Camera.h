#pragma once

#include "LibMath.h"

template <typename T> class Camera {
public:
  Camera();
  virtual ~Camera();

  auto Resize(std::size_t width, std::size_t height) noexcept -> bool;
  auto Resize(std::size_t width, std::size_t height, T nearPlane,
              T farPlane) noexcept -> bool;

  auto Rotate(T du, T dv) -> bool;
  auto Pan(T du, T dv) -> bool;
  auto Zoom(T du) -> bool;

  void SetPerspective(T fov, std::size_t width, std::size_t height, T nearPlane,
                      T farPlane);
  void SetFrustum(T left, T right, T top, T bottom, T nearPlane, T farPlane);

  void SetPosition(const Vec3<T> &position);
  void SetPosition(T x, T y, T z);
  void AddPosition(const Vec3<T> &displacement);
  void AddPosition(T dx, T dy, T dz);

  void SetRotation(T theta, T phi);
  void SetHorizontalRotation(T theta);
  void SetVerticalRotation(T phi);
  void SetSphericalPosition(T r, T theta, T phi);
  void AddRadius(T dRadius);
  void SetRadius(T radius);
  void SetFieldOfView(T fov);
  void SetNearPlane(T nearPlane);
  void SetFarPlane(T farPlane);
  void SetMinRadius(T minRadius);
  void SetMaxRadius(T maxRadius);

  void SetRotationSensitivity(T sen);
  void SetPanSensitivity(T sen);
  void SetZoomSensitivity(T sen);
  void SetScopeSensitivity(T sen);

  void Reset();
  void ResetPlanes();
  void ResetView();
  void ResetMatrices();
  void ResetSensitivities();

  auto GetR() const -> T { return r; }
  auto GetTheta() const -> T { return theta; }
  auto GetPhi() const -> T { return phi; }

  void SetR(T newValue) {
    r = newValue;
    Compile();
  }
  void SetTheta(T newValue) {
    theta = newValue;
    Compile();
  }
  void SetPhi(T newValue) {
    phi = newValue;
    Compile();
  }

  auto GetRotationSensitivity() const -> T;
  auto GetPanSensitivity() const -> T;
  auto GetZoomSensitivity() const -> T;
  auto GetScopeSensitivity() const -> T;

  auto GetViewDirection() -> Vec3<T>;
  auto GetRightDirection() -> Vec3<T>;
  auto GetLeftDirection() -> Vec3<T>;
  auto GetUpDirection() -> Vec3<T>;
  auto GetDownDirection() -> Vec3<T>;

  auto GetEye() const -> const Vec3<T> &;
  auto GetLookAt() const -> const Vec3<T> &;
  auto GetUp() const -> const Vec3<T> &;
  auto GetFOV() const -> const T &;

  std::size_t GetWidth() const;
  auto GetHeight() const -> std::size_t;
  auto GetNear() const -> const T &;
  auto GetFar() const -> const T &;

  auto GetViewMatrix() -> const Mat4<T> &;
  auto GetProjectionMatrix() -> const Mat4<T> &;

  auto ToViewMatrix() -> Mat4<T>;
  auto ToProjectionMatrix() -> Mat4<T>;

  static auto LookAt(const Vec3<T> &eye, const Vec3<T> &lookAt,
                     const Vec3<T> &up) -> Mat4<T>;
  static auto LookAt(T eyex, T eyey, T eyez, T atx, T aty, T atz, T upx, T upy,
                     T upz) -> Mat4<T>;

  static auto PerspectiveMatrix(T fov, std::size_t width, std::size_t height,
                                T nearPlane, T farPlane) noexcept -> Mat4<T>;
  static auto OrthographicMatrix(T left, T right, T bottom, T top, T nearPlane,
                                 T farPlane) noexcept -> Mat4<T>;

  static auto SphereicalToCartesian(T r, T theta, T phi) -> Vec3<T>;
  static auto SphereicalToCartesian_dTheta(T r, T theta, T phi) -> Vec3<T>;
  static auto SphereicalToCartesian_dPhi(T r, T theta, T phi) -> Vec3<T>;
  static auto SphereicalToCartesian_dPhiCrossdTheta(T r, T theta, T phi)
      -> Vec3<T>;

protected:
  void Compile() noexcept;

protected:
  Mat4<T> viewMatrix;
  Mat4<T> projectionMatrix;
  T nearPlane, farPlane;

  std::size_t width, height;
  Vec3<T> eye;
  Vec3<T> lookAt;
  Vec3<T> up;

  T fov, aspectRatio;
  T minRadius, maxRadius;

  T r, theta, phi;
  Vec3<T> displacement;

  T panSensitivity;
  T zoomSensitivity;
  T rotationSensitivity;
  T scopeSensitivity;
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

template <typename T> auto Camera<T>::GetFOV() const -> const T & {
  return this->fov;
}

template <typename T> Camera<T>::Camera() {
  this->viewMatrix = Mat4<T>::Identity();
  this->projectionMatrix = Mat4<T>::Identity();

  this->eye = Vec3<T>::UnitZ();
  this->lookAt = Vec3<T>::Zero();
  this->up = Vec3<T>::UnitY();

  this->nearPlane = T(DEFAULT_NEAR_PLANE);
  this->farPlane = T(DEFAULT_FAR_PLANE);

  this->rotationSensitivity = T(DEFAULT_ROTATION_SENSITIVITY);
  this->panSensitivity = T(DEFAULT_PAN_SENSITIVITY);
  this->zoomSensitivity = T(DEFAULT_ZOOM_SENSITIVITY);
  this->scopeSensitivity = T(DEFAULT_SCOPE_SENSITIVITY);

  this->r = T(1);
  this->theta = T(DEFAULT_THETA);
  this->phi = T(DEFAULT_PHI);
  this->fov = T(DEFAULT_FOV);
  this->aspectRatio = T(DEFAULT_ASPECT_RATIO);
  this->minRadius = T(DEFAULT_MIN_RADIUS);
  this->maxRadius = T(DEFAULT_MAX_RADIUS);
  this->displacement = Vec3<T>::Zero();
}

template <typename T> Camera<T>::~Camera() = default;

template <typename T>
auto Camera<T>::Resize(std::size_t width, std::size_t height) noexcept -> bool {
  this->SetPerspective(this->fov, width, height, this->nearPlane,
                       this->farPlane);
  this->Compile();
  return true;
}

template <typename T>
auto Camera<T>::Resize(std::size_t width, std::size_t height, T nearPlane,
                       T farPlane) noexcept -> bool {
  this->SetPerspective(this->fov, width, height, this->nearPlane,
                       this->farPlane);
  this->Compile();
  return true;
}

template <typename T> auto Camera<T>::Rotate(T du, T dv) -> bool {
  this->theta -= (du * this->rotationSensitivity);
  this->phi += (dv * this->rotationSensitivity);
  this->Compile();
  return true;
}

template <typename T> auto Camera<T>::Pan(T du, T dv) -> bool {
  Vec3<T> uDir = this->GetLeftDirection();
  Vec3<T> vDir = this->GetDownDirection();

  Vec3<T> uDisp = (du * this->panSensitivity) * uDir;
  Vec3<T> vDisp = (dv * this->panSensitivity) * vDir;
  Vec3<T> panDisp = uDisp + vDisp;

  this->displacement += panDisp;
  this->Compile();
  return true;
}

template <typename T> auto Camera<T>::Zoom(T dr) -> bool {
  if (this->r + dr > this->maxRadius) {
    this->r = this->maxRadius;
    this->Compile();
    return false;
  } else if (this->r + dr < this->minRadius) {
    this->r = this->minRadius;
    this->Compile();
    return false;
  } else {
    this->r += dr;
    this->Compile();
    return true;
  }

  return false;
}

template <typename T>
void Camera<T>::SetPerspective(T fov, std::size_t width, std::size_t height,
                               T nearPlane, T farPlane) {
  if (fov > T(MAX_FOV))
    this->fov = T(MAX_FOV);
  else if (fov < T(MIN_FOV))
    this->fov = T(MIN_FOV);
  else
    this->fov = fov;

  this->width = width;
  this->height = height;
  this->nearPlane = nearPlane;
  this->farPlane = farPlane;

  T aspectRatio = static_cast<T>(width) / static_cast<T>(height);
  T ymax = nearPlane * std::tan(fov * T(PI) / T(360));
  T xmax = ymax * aspectRatio;
  this->SetFrustum(-xmax, xmax, ymax, -ymax, nearPlane, farPlane);
}

template <typename T>
void Camera<T>::SetFrustum(T left, T right, T top, T bottom, T nearPlane,
                           T farPlane) {
  this->projectionMatrix.setZero();

  T temp, temp2, temp3, temp4;
  temp = T(2) * nearPlane;
  temp2 = right - left;
  temp3 = top - bottom;
  temp4 = farPlane - nearPlane;

  this->projectionMatrix(0) = temp / temp2;
  this->projectionMatrix(5) = temp / temp3;
  this->projectionMatrix(8) = (right + left) / temp2;
  this->projectionMatrix(9) = (top + bottom) / temp3;
  this->projectionMatrix(10) = (-farPlane - nearPlane) / temp4;
  this->projectionMatrix(11) = T(-1);
  this->projectionMatrix(14) = (-temp * farPlane) / temp4;
}

template <typename T> void Camera<T>::SetPosition(const Vec3<T> &pos) {
  this->position.x() = pos.x();
  this->position.y() = pos.y();
  this->position.z() = pos.z();
  this->Compile();
}

template <typename T> void Camera<T>::SetPosition(T x, T y, T z) {
  this->position.x() = x;
  this->position.y() = y;
  this->position.z() = z;
  this->Compile();
}

template <typename T> void Camera<T>::AddPosition(const Vec3<T> &displacement) {
  this->position += displacement;
  this->Compile();
}

template <typename T> void Camera<T>::AddPosition(T dx, T dy, T dz) {
  this->displacement.x() += dx;
  this->displacement.y() += dy;
  this->displacement.z() += dz;
  this->Compile();
}

template <typename T> void Camera<T>::SetRotation(T theta, T phi) {
  this->theta = theta;
  this->phi = phi;
  this->Compile();
}

template <typename T> void Camera<T>::SetHorizontalRotation(T theta) {
  this->theta = theta;
  this->Compile();
}

template <typename T> void Camera<T>::SetVerticalRotation(T phi) {
  this->phi = phi;
  this->Compile();
}

template <typename T>
void Camera<T>::SetSphericalPosition(T r, T theta, T phi) {
  this->r = r;
  this->theta = theta;
  this->phi = phi;
  this->Compile();
}

template <typename T> void Camera<T>::AddRadius(T dRadius) {
  this->r += dRadius;
  this->Compile();
}

template <typename T> void Camera<T>::SetRadius(T radius) {
  this->r = radius;
  this->Compile();
}

template <typename T> void Camera<T>::SetFieldOfView(T fov) {
  if (fov > T(MAX_FOV))
    this->fov = T(MAX_FOV);
  else if (fov < T(MIN_FOV))
    this->fov = T(MIN_FOV);
  else
    this->fov = fov;
  this->SetPerspective(this->fov, this->aspectRatio, this->nearPlane,
                       this->farPlane);
}

template <typename T> void Camera<T>::SetNearPlane(T nearPlane) {
  this->nearPlane = nearPlane;
  this->SetPerspective(this->fov, this->aspectRatio, this->nearPlane,
                       this->farPlane);
}

template <typename T> void Camera<T>::SetFarPlane(T farPlane) {
  this->farPlane = farPlane;
  this->SetPerspective(this->fov, this->aspectRatio, this->nearPlane,
                       this->farPlane);
}

template <typename T> void Camera<T>::SetMinRadius(T minRadius) {
  this->minRadius = minRadius;
}

template <typename T> void Camera<T>::SetMaxRadius(T maxRadius) {
  this->maxRadius = maxRadius;
}

template <typename T> void Camera<T>::SetRotationSensitivity(T sen) {
  this->rotationSensitivity = sen;
}

template <typename T> void Camera<T>::SetPanSensitivity(T sen) {
  this->panSensitivity = sen;
}

template <typename T> void Camera<T>::SetZoomSensitivity(T sen) {
  this->zoomSensitivity = sen;
}

template <typename T> void Camera<T>::SetScopeSensitivity(T sen) {
  this->scopeSensitivity = sen;
}

template <typename T> void Camera<T>::Reset() {
  this->ResetMatrices();
  this->ResetPlanes();
  this->ResetView();
  this->ResetSensitivities();
  this->Compile();
}

template <typename T> void Camera<T>::ResetPlanes() {
  this->nearPlane = T(DEFAULT_NEAR_PLANE);
  this->farPlane = T(DEFAULT_FAR_PLANE);
}

template <typename T> void Camera<T>::ResetView() {
  this->eye = Vec3<T>::UnitZ();
  this->lookAt = Vec3<T>::Zero();
  this->up = Vec3<T>::UnitY();
}

template <typename T> void Camera<T>::ResetMatrices() {
  this->viewMatrix = Mat4<T>::Identity();
  this->projectionMatrix = Mat4<T>::Identity();
}

template <typename T> void Camera<T>::ResetSensitivities() {
  this->rotationSensitivity = T(DEFAULT_ROTATION_SENSITIVITY);
  this->panSensitivity = T(DEFAULT_PAN_SENSITIVITY);
  this->zoomSensitivity = T(DEFAULT_ZOOM_SENSITIVITY);
  this->scopeSensitivity = T(DEFAULT_SCOPE_SENSITIVITY);
}

template <typename T> auto Camera<T>::GetRotationSensitivity() const -> T {
  return this->rotationSensitivity;
}

template <typename T> auto Camera<T>::GetPanSensitivity() const -> T {
  return this->panSensitivity;
}

template <typename T> auto Camera<T>::GetZoomSensitivity() const -> T {
  return this->zoomSensitivity;
}

template <typename T> auto Camera<T>::GetScopeSensitivity() const -> T {
  return this->scopeSensitivity;
}

template <typename T> inline auto Camera<T>::GetViewDirection() -> Vec3<T> {
  this->Compile();
  return (this->lookAt - this->eye).normalized();
}

template <typename T> inline auto Camera<T>::GetRightDirection() -> Vec3<T> {
  this->Compile();
  Vec3<T> dir = (this->lookAt - this->eye).normalized();
  return (this->up.cross(dir)).normalized();
}

template <typename T> inline auto Camera<T>::GetLeftDirection() -> Vec3<T> {
  this->Compile();
  Vec3<T> dir = (this->lookAt - this->eye).normalized();
  return (dir.cross(this->up)).normalized();
}

template <typename T> inline auto Camera<T>::GetUpDirection() -> Vec3<T> {
  this->Compile();
  return this->up;
}

template <typename T> inline auto Camera<T>::GetDownDirection() -> Vec3<T> {
  this->Compile();
  return -this->up;
}

template <typename T> auto Camera<T>::GetEye() const -> const Vec3<T> & {
  return this->eye;
}

template <typename T> auto Camera<T>::GetLookAt() const -> const Vec3<T> & {
  return this->lookAt;
}

template <typename T> auto Camera<T>::GetUp() const -> const Vec3<T> & {
  return this->up;
}

template <typename T> auto Camera<T>::GetWidth() const -> std::size_t {
  return this->width;
}

template <typename T> auto Camera<T>::GetHeight() const -> std::size_t {
  return this->height;
}

template <typename T> auto Camera<T>::GetNear() const -> const T & {
  return this->nearPlane;
}

template <typename T> auto Camera<T>::GetFar() const -> const T & {
  return this->farPlane;
}

template <typename T> auto Camera<T>::GetViewMatrix() -> const Mat4<T> & {
  this->Compile();
  return this->viewMatrix;
}

template <typename T> auto Camera<T>::GetProjectionMatrix() -> const Mat4<T> & {
  this->Compile();
  return this->projectionMatrix;
}

template <typename T> auto Camera<T>::ToViewMatrix() -> Mat4<T> {
  this->Compile();
  return this->viewMatrix;
}

template <typename T> auto Camera<T>::ToProjectionMatrix() -> Mat4<T> {
  this->Compile();
  return this->projectionMatrix;
}

template <typename T>
auto Camera<T>::LookAt(const Vec3<T> &eye, const Vec3<T> &lookAt,
                       const Vec3<T> &up) -> Mat4<T> {
  return Camera<T>::LookAt(eye.x(), eye.y(), eye.z(), lookAt.x(), lookAt.y(),
                           lookAt.z(), up.x(), up.y(), up.z());
}

template <typename T>
auto Camera<T>::LookAt(T eyex, T eyey, T eyez, T atx, T aty, T atz, T upx,
                       T upy, T upz) -> Mat4<T> {
  Mat4<T> matrix;
  Vec3<T> x, y, z;
  Vec3<T> eye(eyex, eyey, eyez);

  y = Vec3<T>(upx, upy, upz);
  z = Vec3<T>(atx - eyex, aty - eyey, atz - eyez);
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

  matrix(3, 0) = T(0);
  matrix(3, 1) = T(0);
  matrix(3, 2) = T(0);
  matrix(3, 3) = T(1);

  return matrix;
}

template <typename T>
auto Camera<T>::PerspectiveMatrix(T fov, std::size_t width, std::size_t height,
                                  T nearPlane, T farPlane) noexcept -> Mat4<T> {
  Mat4<T> proj;
  proj.setZero();

  T aspectRatio = static_cast<T>(width) / static_cast<T>(height);
  T ymax = nearPlane * std::tan(fov * T(PI) / T(360));
  T xmax = ymax * aspectRatio;
  T left = -xmax;
  T right = xmax;
  T top = ymax;
  T bottom = -ymax;

  T temp, temp2, temp3, temp4;
  temp = T(2) * nearPlane;
  temp2 = right - left;
  temp3 = top - bottom;
  temp4 = farPlane - nearPlane;

  proj(0) = temp / temp2;
  proj(5) = temp / temp3;
  proj(8) = (right + left) / temp2;
  proj(9) = (top + bottom) / temp3;
  proj(10) = (-farPlane - nearPlane) / temp4;
  proj(11) = T(-1);
  proj(14) = (-temp * farPlane) / temp4;

  return proj;
}

template <typename T>
auto Camera<T>::OrthographicMatrix(T left, T right, T bottom, T top,
                                   T nearPlane, T farPlane) noexcept
    -> Mat4<T> {
  Mat4<T> proj;

  T x = T(2) / (right - left);
  T y = T(2) / (top - bottom);
  T z = T(-2) / (farPlane - nearPlane);
  T tx = (right + left) / (right - left);
  T ty = (top + bottom) / (top - bottom);
  T tz = (farPlane + nearPlane) / (farPlane - nearPlane);

  proj.setZero();

  proj(0, 0) = x;
  proj(1, 1) = y;
  proj(2, 2) = z;
  proj(3, 3) = T(1);

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
template <typename T>
inline auto Camera<T>::SphereicalToCartesian(T r, T theta, T phi) -> Vec3<T> {
  Vec3<T> result;

  T sinPhi = std::sin(phi);
  T cosPhi = std::cos(phi);
  T sinTheta = std::sin(theta);
  T cosTheta = std::cos(theta);

  result.x() = r * (cosTheta * sinPhi);
  result.z() = r * (sinTheta * sinPhi);
  result.y() = r * cosPhi;
  return result;
}

/* Unswapped: Rt(r, t, p) = -rsin(phi)sin(theta)i + rsin(phi)cos(theta)j + 0k */
template <typename T>
inline auto Camera<T>::SphereicalToCartesian_dTheta(T r, T theta, T phi)
    -> Vec3<T> {
  Vec3<T> result;

  T sinPhi = std::sin(phi);
  T cosPhi = std::cos(phi);
  T sinTheta = std::sin(theta);
  T cosTheta = std::cos(theta);

  result.x() = -r * (sinPhi * sinTheta);
  result.z() = r * (sinPhi * cosTheta);
  result.y() = T(0);
  return result;
}

/* Unswapped: Rp(r, t, p) = rcos(phi)cos(theta)i + rcos(phi)sin(theta)j -
 * rsin(phi)k */
template <typename T>
inline auto Camera<T>::SphereicalToCartesian_dPhi(T r, T theta, T phi)
    -> Vec3<T> {
  Vec3<T> result;

  T sinPhi = std::sin(phi);
  T cosPhi = std::cos(phi);
  T sinTheta = std::sin(theta);
  T cosTheta = std::cos(theta);

  result.x() = r * (cosPhi * cosTheta);
  result.z() = r * (cosPhi * sinTheta);
  result.y() = -r * sinPhi;
  return result;
}

/* Rp X Rt = r^2 * sin^2(phi)cos(theta)i + r^2 * sin^2(phi)sin(theta)j + r^2 *
 * sin(phi)cos(phi)k */
template <typename T>
inline auto Camera<T>::SphereicalToCartesian_dPhiCrossdTheta(T r, T theta,
                                                             T phi) -> Vec3<T> {
  Vec3<T> result;

  T rs = (r * r);
  T sinPhi = std::sin(phi);
  T cosPhi = std::cos(phi);
  T sinTheta = std::sin(theta);
  T cosTheta = std::cos(theta);

  result.x() = -rs * ((sinPhi * sinPhi) * cosTheta);
  result.y() = -rs * ((sinPhi * sinPhi) * sinTheta);
  result.z() = -rs * sinPhi * cosPhi;
  return result;
}

template <typename T> inline void Camera<T>::Compile() noexcept {
  this->lookAt = Vec3<T>::Zero();
  this->eye = Camera<T>::SphereicalToCartesian(this->r, this->theta, this->phi);
  this->up =
      Camera<T>::SphereicalToCartesian_dPhi(this->r, this->theta, this->phi)
          .normalized();

  //--------------------------------------------------------------------------------
  // Invert the up direction (since the spherical coordinates have phi
  // increasing downwards. Therefore we would like to have the (vector)
  // direction of the derivative inversed.
  //--------------------------------------------------------------------------------
  this->up *= T(-1.0);

  this->lookAt += this->displacement;
  this->eye += this->displacement;

  this->viewMatrix = Camera<T>::LookAt(this->eye, this->lookAt, this->up);
}
