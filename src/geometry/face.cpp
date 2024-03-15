#include "geometry/face.h"

#include <glm/geometric.hpp>
namespace gfx {

Face::Face(const Vertex& a, const Vertex& b, const Vertex& c) : a_(a), b_(b), c_(c) {
  const auto edge01 = b.position() - a.position();
  const auto edge02 = c.position() - a.position();
  const auto normal = glm::cross(edge01, edge02);
  const auto normal_magnitude = glm::length(normal);
  assert(normal_magnitude > 0.0f);  // ensure face vertices are not collinear
  area_ = 0.5f * normal_magnitude;  // NOLINT(*-magic-numbers)
  normal_ = normal / normal_magnitude;
}

}  // namespace gfx
