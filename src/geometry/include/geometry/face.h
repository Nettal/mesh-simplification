#ifndef SRC_GEOMETRY_INCLUDE_GEOMETRY_FACE_H_
#define SRC_GEOMETRY_INCLUDE_GEOMETRY_FACE_H_

#include <cassert>
#include <memory>
#include <vector>

#include <glm/vec3.hpp>

#include "HalfEdgeKey.h"
#include "geometry/vertex.h"

namespace gfx {

/** \brief A triangle face in a half-edge mesh. */
class Face {
public:
  /**
   * \brief Initializes a face.
   * \param v0,v1,v2 The face vertices in counter-clockwise order.
   */
  Face(const Vertex& a, const Vertex& b, const Vertex& c);

  [[nodiscard]] std::vector<Vertex> poses() const noexcept { return {a_, b_, c_}; }

  /** \brief Gets the face normal. */
  [[nodiscard]] const glm::vec3& normal() const noexcept { return normal_; }

  /** \brief Gets the face area. */
  [[nodiscard]] float area() const noexcept { return area_; }

  [[nodiscard]] std::vector<HalfEdgeKey> edges() const noexcept {
    auto v1 = HalfEdgeKey(a_.position(), b_.position());
    auto v2 = HalfEdgeKey(b_.position(), c_.position());
    auto v3 = HalfEdgeKey(c_.position(), a_.position());
    return {v1, v2, v3};
  }

  [[nodiscard]] std::vector<Vertex> vertex() const noexcept { return {a_, b_, c_}; }

  [[nodiscard]] Vertex getVertex(glm::vec3 p) const noexcept {
    if (p == a_.position()) return a_;
    if (p == b_.position()) return b_;
    if (p == c_.position()) return c_;
    assert(0);
  }

  Vertex a_;
  Vertex b_;
  Vertex c_;
private:
  glm::vec3 normal_{};
  float area_{};
};
struct FaceHash {
  std::size_t operator()(const Face& type) const {
    int64_t result = 1;
    result = 31 * result + *((int*)(&type.a_.position().x));
    result = 31 * result + *((int*)(&type.a_.position().y));
    result = 31 * result + *((int*)(&type.a_.position().z));
    result = 31 * result + *((int*)(&type.b_.position().x));
    result = 31 * result + *((int*)(&type.b_.position().y));
    result = 31 * result + *((int*)(&type.b_.position().z));
    result = 31 * result + *((int*)(&type.c_.position().x));
    result = 31 * result + *((int*)(&type.c_.position().y));
    result = 31 * result + *((int*)(&type.c_.position().z));
    result = 31 * result + *((int*)(&type.normal().x));
    result = 31 * result + *((int*)(&type.normal().y));
    result = 31 * result + *((int*)(&type.normal().z));
    return result;
  }
};

struct FaceEqual {
  bool operator()(const Face& lhs, const Face& rhs) const {
    return lhs.a_.position() == rhs.a_.position() && lhs.b_.position() == rhs.b_.position()
           && lhs.c_.position() == rhs.c_.position() && lhs.normal() == rhs.normal();
  }
};
}  // namespace gfx

#endif  // SRC_GEOMETRY_INCLUDE_GEOMETRY_FACE_H_
