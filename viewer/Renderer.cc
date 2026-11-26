#include <Renderer.h>

#include <glad/glad.h>

#include <cstddef>
#include <stdexcept>
#include <string>

namespace viewer {
namespace {

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
    : colorProgram(Link(kColorVertex, kColorFragment)),
      litProgram(Link(kLitVertex, kLitFragment)) {
  for (Stream *stream : {&lines, &points, &triangles}) {
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
  glBindVertexArray(0);
}

Renderer::~Renderer() {
  for (Stream *stream : {&lines, &points, &triangles}) {
    glDeleteBuffers(1, &stream->vbo);
    glDeleteVertexArrays(1, &stream->vao);
  }
  glDeleteProgram(colorProgram);
  glDeleteProgram(litProgram);
}

void Renderer::Draw(const FrameGeometry &geometry, const Eigen::Matrix4f &view,
                    const Eigen::Matrix4f &projection, int width,
                    int height) {
  glViewport(0, 0, width, height);
  // The old drivers' background.
  glClearColor(0.15f, 0.15f, 0.15f, 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
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
