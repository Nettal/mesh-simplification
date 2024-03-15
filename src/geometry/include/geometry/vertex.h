#ifndef SRC_GEOMETRY_INCLUDE_GEOMETRY_VERTEX_H_
#define SRC_GEOMETRY_INCLUDE_GEOMETRY_VERTEX_H_

#include <cassert>
#include <concepts>
#include <cstdint>
#include <memory>
#include <optional>

#include <glm/vec3.hpp>

#define var auto
#define null nullptr
namespace gfx {
class ProcessingVertex;

/** \brief A vertex in a half-edge mesh. */
class Vertex {
public:
  /**
   * \brief Initializes a vertex.
   * \param position The vertex position.
   */
  explicit Vertex(const glm::vec3& position) noexcept : position_{position} {}

  /** \brief Gets the vertex position. */
  [[nodiscard]] const glm::vec3& position() const noexcept { return position_; }

  struct Vec3Hash {
    std::size_t operator()(const glm::vec3 vec3) const noexcept {
      int64_t result = 1;
      result = 31 * result + *((int*)(&vec3.x));
      result = 31 * result + *((int*)(&vec3.y));
      result = 31 * result + *((int*)(&vec3.z));
      return result;
    }
  };
  struct Vec3Equal {
    bool operator()(const glm::vec3 lhs, const glm::vec3 rhs) const noexcept {
      return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
    }
  };

private:
  glm::vec3 position_;
};

}  // namespace gfx

#endif  // SRC_GEOMETRY_INCLUDE_GEOMETRY_VERTEX_H_
