#ifndef SRC_GEOMETRY_INCLUDE_GEOMETRY_HALF_EDGE_H_
#define SRC_GEOMETRY_INCLUDE_GEOMETRY_HALF_EDGE_H_

#include <cassert>
#include <memory>
#include <unordered_set>

#include "half_edge_mesh.h"
#include "geometry/face.h"
#include "geometry/vertex.h"

namespace gfx {

class ProcessingVertex {
public:
  explicit ProcessingVertex(auto pos) noexcept : pos_(pos) {}

  void addFace(Face f) noexcept { faces_.emplace(f); }

  void removeFace(Face f) noexcept { faces_.erase(f); }

  [[nodiscard]] auto faces() const noexcept { return faces_; }

  glm::vec3 pos() const { return pos_; }

private:
  glm::vec3 pos_;
  face_set faces_;
};

}  // namespace gfx

#endif  // SRC_GEOMETRY_INCLUDE_GEOMETRY_HALF_EDGE_H_
