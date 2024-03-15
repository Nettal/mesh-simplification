#ifndef SRC_GEOMETRY_INCLUDE_GEOMETRY_MESH_SIMPLIFIER_H_
#define SRC_GEOMETRY_INCLUDE_GEOMETRY_MESH_SIMPLIFIER_H_

#include "glm/glm.hpp"
#include "unordered_map"
#include "vertex.h"

namespace gfx {
class Device;
class Mesh;

namespace mesh {

using vec3_mat4_map = std::unordered_map<glm::vec3, glm::mat4, Vertex::Vec3Hash, Vertex::Vec3Equal>;
/**
 * \brief Reduces the number of triangles in a mesh.
 * \param device The graphics device used to load the reconstructed mesh data into GPU memory.
 * \param mesh The mesh to simplify.
 * \param rate The percentage of triangles to be removed (e.g., .95 indicates 95% of triangles should be removed).
 * \return A triangle mesh with \p rate percent of triangles removed from \p mesh.
 * \see docs/surface_simplification for a description of this mesh simplification algorithm.
 */
Mesh Simplify(const Device& device, const Mesh& mesh, float rate);
}  // namespace mesh
}  // namespace gfx

#endif  // SRC_GEOMETRY_INCLUDE_GEOMETRY_MESH_SIMPLIFIER_H_
