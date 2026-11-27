#include <AlbedoTexture.h>
#include <Renderer.h>

#include <glad/glad.h>

#include <Eigen/LU>

#include <cstddef>
#include <stdexcept>
#include <string>

namespace viewer {
namespace {

// One triangle that covers the screen, shaded from horizon to zenith.
constexpr const char *kBackgroundVertex = R"(#version 410 core
out float vHeight;
void main() {
  vec2 corner = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
  vHeight = corner.y;
  gl_Position = vec4(corner * 2.0 - 1.0, 1.0, 1.0);
}
)";

constexpr const char *kBackgroundFragment = R"(#version 410 core
in float vHeight;
out vec4 fragColor;
void main() {
  vec3 bottom = vec3(0.09, 0.09, 0.10);
  vec3 top = vec3(0.24, 0.25, 0.28);
  fragColor = vec4(mix(bottom, top, smoothstep(0.0, 1.0, vHeight)), 1.0);
}
)";

constexpr const char *kColorVertex = R"(#version 410 core
uniform mat4 view;
uniform mat4 projection;
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 color;
layout(location = 2) in float pointSize;
out vec3 vColor;
void main() {
  vColor = color;
  gl_PointSize = pointSize;
  gl_Position = projection * view * vec4(position, 1.0);
}
)";

constexpr const char *kColorFragment = R"(#version 410 core
uniform bool roundPoints;
in vec3 vColor;
out vec4 fragColor;
void main() {
  if (roundPoints && length(gl_PointCoord - vec2(0.5)) > 0.5) {
    discard;
  }
  fragColor = vec4(vColor, 1.0);
}
)";

constexpr const char *kLitVertex = R"(#version 410 core
uniform mat4 view;
uniform mat4 projection;
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec3 color;
out vec3 vNormal;
out vec3 vColor;
void main() {
  vNormal = mat3(view) * normal;
  vColor = color;
  gl_Position = projection * view * vec4(position, 1.0);
}
)";

// A headlight: ambient plus two-sided diffuse toward the viewer.
constexpr const char *kLitFragment = R"(#version 410 core
in vec3 vNormal;
in vec3 vColor;
out vec4 fragColor;
void main() {
  float diffuse = abs(normalize(vNormal).z);
  fragColor = vec4(vColor * (0.35 + 0.65 * diffuse), 1.0);
}
)";

constexpr const char *kSurfaceVertex = R"(#version 410 core
uniform mat4 view;
uniform mat4 projection;
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec3 restPosition;
layout(location = 3) in vec3 restNormal;
out vec3 vPosition;
out vec3 vNormal;
out vec3 vRestPosition;
out vec3 vRestNormal;
void main() {
  vPosition = position;
  vNormal = normal;
  vRestPosition = restPosition;
  vRestNormal = restNormal;
  gl_Position = projection * view * vec4(position, 1.0);
}
)";

// Lighting is linear: the albedo texture is sRGB, so sampling decodes it, and
// the sum is ACES-tonemapped and gamma encoded at the end. The texture is
// projected from the rest pose so it stays on the material as it deforms.
constexpr const char *kSurfaceFragment = R"(#version 410 core
uniform sampler2D albedo;
uniform float textureScale;
uniform vec3 eye;
uniform vec3 lightPosition;
uniform vec3 lightColor;
uniform float lightIntensity;
uniform float ambientStrength;
in vec3 vPosition;
in vec3 vNormal;
in vec3 vRestPosition;
in vec3 vRestNormal;
out vec4 fragColor;

const float kLightRange = 0.5; // keeps the falloff finite at the light
const float kSpecular = 0.2;
const float kShininess = 20.0;
const vec3 kSky = vec3(0.62, 0.68, 0.80);
const vec3 kGround = vec3(0.34, 0.28, 0.23);

vec3 Triplanar(vec3 p, vec3 n) {
  vec3 w = pow(abs(n), vec3(4.0));
  w /= w.x + w.y + w.z + 1e-5;
  p *= textureScale;
  return w.x * texture(albedo, p.zy).rgb + w.y * texture(albedo, p.xz).rgb +
         w.z * texture(albedo, p.xy).rgb;
}

vec3 AcesFit(vec3 x) {
  return clamp(x * (2.51 * x + 0.03) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0);
}

void main() {
  vec3 n = normalize(vNormal);
  vec3 base = Triplanar(vRestPosition, normalize(vRestNormal));

  vec3 toLight = lightPosition - vPosition;
  float distance2 = dot(toLight, toLight);
  vec3 l = toLight * inversesqrt(distance2);
  vec3 radiance =
      lightColor * lightIntensity / (distance2 + kLightRange * kLightRange);
  vec3 h = normalize(l + normalize(eye - vPosition));
  float lambert = max(dot(n, l), 0.0);
  float specular = kSpecular * pow(max(dot(n, h), 0.0), kShininess) * lambert;
  vec3 ambient = ambientStrength * mix(kGround, kSky, 0.5 + 0.5 * n.y) * base;

  vec3 color = (base * lambert + specular) * radiance + ambient;
  fragColor = vec4(pow(AcesFit(color), vec3(1.0 / 2.2)), 1.0);
}
)";

auto Compile(GLenum stage, const char *source) -> GLuint {
  const GLuint shader = glCreateShader(stage);
  glShaderSource(shader, 1, &source, nullptr);
  glCompileShader(shader);
  GLint ok = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
  if (ok == 0) {
    std::string log(1024, '\0');
    glGetShaderInfoLog(shader, static_cast<GLsizei>(log.size()), nullptr,
                       log.data());
    throw std::runtime_error("shader compile failed: " + log);
  }
  return shader;
}

auto Link(const char *vertex, const char *fragment) -> GLuint {
  const GLuint vs = Compile(GL_VERTEX_SHADER, vertex);
  const GLuint fs = Compile(GL_FRAGMENT_SHADER, fragment);
  const GLuint program = glCreateProgram();
  glAttachShader(program, vs);
  glAttachShader(program, fs);
  glLinkProgram(program);
  glDeleteShader(vs);
  glDeleteShader(fs);
  GLint ok = 0;
  glGetProgramiv(program, GL_LINK_STATUS, &ok);
  if (ok == 0) {
    throw std::runtime_error("shader link failed");
  }
  return program;
}

struct Attribute {
  GLint components;
  std::size_t offset;
};

template <typename Vertex, std::size_t N>
void Describe(GLuint vao, GLuint vbo, const Attribute (&attributes)[N]) {
  glBindVertexArray(vao);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  for (GLuint ii = 0; ii < N; ++ii) {
    glEnableVertexAttribArray(ii);
    glVertexAttribPointer(ii, attributes[ii].components, GL_FLOAT, GL_FALSE,
                          sizeof(Vertex),
                          reinterpret_cast<const void *>(attributes[ii].offset));
  }
}

template <typename Vertex>
void Upload(GLuint vbo, const std::vector<Vertex> &vertices) {
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  // Orphan and refill: the buffer is rewritten every frame.
  glBufferData(GL_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)),
               vertices.data(), GL_STREAM_DRAW);
}

auto UploadAlbedo() -> GLuint {
  constexpr std::uint32_t kSeed = 7;
  constexpr int kSize = 512;
  const Albedo albedo = GenerateAlbedo(kSeed, kSize);
  GLuint texture = 0;
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  // GL_SRGB8 decodes to linear on sampling and keeps mip averaging linear.
  glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB8, kSize, kSize, 0, GL_RGB,
               GL_UNSIGNED_BYTE, albedo.pixels.data());
  glGenerateMipmap(GL_TEXTURE_2D);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                  GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glBindTexture(GL_TEXTURE_2D, 0);
  return texture;
}

void SetCamera(GLuint program, const Eigen::Matrix4f &view,
               const Eigen::Matrix4f &projection) {
  glUseProgram(program);
  glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE,
                     view.data());
  glUniformMatrix4fv(glGetUniformLocation(program, "projection"), 1, GL_FALSE,
                     projection.data());
}

} // namespace

Renderer::Renderer()
    : backgroundProgram(Link(kBackgroundVertex, kBackgroundFragment)),
      colorProgram(Link(kColorVertex, kColorFragment)),
      litProgram(Link(kLitVertex, kLitFragment)),
      surfaceProgram(Link(kSurfaceVertex, kSurfaceFragment)),
      albedoTexture(UploadAlbedo()) {
  glGenVertexArrays(1, &backgroundVao);
  for (Stream *stream : {&lines, &points, &triangles, &surfaces}) {
    glGenVertexArrays(1, &stream->vao);
    glGenBuffers(1, &stream->vbo);
  }
  constexpr Attribute kColorAttributes[] = {
      {3, offsetof(ColorVertex, position)},
      {3, offsetof(ColorVertex, color)},
      {1, offsetof(ColorVertex, pointSize)}};
  Describe<ColorVertex>(lines.vao, lines.vbo, kColorAttributes);
  Describe<ColorVertex>(points.vao, points.vbo, kColorAttributes);
  constexpr Attribute kLitAttributes[] = {{3, offsetof(LitVertex, position)},
                                          {3, offsetof(LitVertex, normal)},
                                          {3, offsetof(LitVertex, color)}};
  Describe<LitVertex>(triangles.vao, triangles.vbo, kLitAttributes);
  constexpr Attribute kSurfaceAttributes[] = {
      {3, offsetof(SurfaceVertex, position)},
      {3, offsetof(SurfaceVertex, normal)},
      {3, offsetof(SurfaceVertex, restPosition)},
      {3, offsetof(SurfaceVertex, restNormal)}};
  Describe<SurfaceVertex>(surfaces.vao, surfaces.vbo, kSurfaceAttributes);
  // The element buffer binding is VAO state, so bind it while surfaces.vao is.
  glGenBuffers(1, &surfaceIndexBuffer);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, surfaceIndexBuffer);
  glBindVertexArray(0);
}

Renderer::~Renderer() {
  for (Stream *stream : {&lines, &points, &triangles, &surfaces}) {
    glDeleteBuffers(1, &stream->vbo);
    glDeleteVertexArrays(1, &stream->vao);
  }
  glDeleteBuffers(1, &surfaceIndexBuffer);
  glDeleteVertexArrays(1, &backgroundVao);
  glDeleteTextures(1, &albedoTexture);
  for (GLuint program :
       {backgroundProgram, colorProgram, litProgram, surfaceProgram}) {
    glDeleteProgram(program);
  }
}

void Renderer::DrawSurfaces(const FrameGeometry &geometry,
                            const RenderSettings &settings,
                            const Eigen::Matrix4f &view) {
  const auto uniform = [this](const char *name) {
    return glGetUniformLocation(surfaceProgram, name);
  };
  const Eigen::Vector3f eye = view.inverse().block<3, 1>(0, 3);
  glUniform3fv(uniform("eye"), 1, eye.data());
  glUniform3fv(uniform("lightPosition"), 1, settings.lightPosition.data());
  glUniform3fv(uniform("lightColor"), 1, settings.lightColor.data());
  glUniform1f(uniform("lightIntensity"), settings.lightIntensity);
  glUniform1f(uniform("ambientStrength"), settings.ambientStrength);
  glUniform1f(uniform("textureScale"), settings.textureScale);
  glUniform1i(uniform("albedo"), 0);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, albedoTexture);

  Upload(surfaces.vbo, geometry.surfaceVertices);
  glBindVertexArray(surfaces.vao);
  const auto &indices = geometry.surfaceIndices;
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(indices.size() * sizeof(unsigned int)),
               indices.data(), GL_STREAM_DRAW);
  glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()),
                 GL_UNSIGNED_INT, nullptr);
  glBindTexture(GL_TEXTURE_2D, 0);
}

void Renderer::Draw(const FrameGeometry &geometry,
                    const RenderSettings &settings,
                    const Eigen::Matrix4f &view,
                    const Eigen::Matrix4f &projection, int width, int height) {
  glViewport(0, 0, width, height);
  glClear(GL_DEPTH_BUFFER_BIT);
  glDisable(GL_DEPTH_TEST);
  glUseProgram(backgroundProgram);
  glBindVertexArray(backgroundVao);
  glDrawArrays(GL_TRIANGLES, 0, 3);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_PROGRAM_POINT_SIZE);

  // Surfaces sit slightly behind their wireframe so the edges stay visible.
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1, 1);
  SetCamera(litProgram, view, projection);
  Upload(triangles.vbo, geometry.triangles);
  glBindVertexArray(triangles.vao);
  glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(geometry.triangles.size()));
  SetCamera(surfaceProgram, view, projection);
  glEnable(GL_CULL_FACE); // soft-body surfaces are closed and wound outward
  DrawSurfaces(geometry, settings, view);
  glDisable(GL_CULL_FACE);
  glDisable(GL_POLYGON_OFFSET_FILL);

  SetCamera(colorProgram, view, projection);
  const GLint roundPoints = glGetUniformLocation(colorProgram, "roundPoints");
  glUniform1i(roundPoints, 0);
  Upload(lines.vbo, geometry.lines);
  glBindVertexArray(lines.vao);
  glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(geometry.lines.size()));

  glUniform1i(roundPoints, 1);
  Upload(points.vbo, geometry.points);
  glBindVertexArray(points.vao);
  glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(geometry.points.size()));

  glBindVertexArray(0);
  glUseProgram(0);
}

} // namespace viewer
