#ifndef SRC_GEOMETRY_INCLUDE_GEOMETRY_HALF_EDGE_MESH_H_
#define SRC_GEOMETRY_INCLUDE_GEOMETRY_HALF_EDGE_MESH_H_

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include <glm/mat4x4.hpp>
#include "HalfEdgeKey.h"
#include "face.h"
#include "vertex.h"

namespace gfx {
class Device;
class Face;
class ProcessingVertex;
class Mesh;
class Vertex;

/**
 * \brief An edge centric data structure used to represent a triangle mesh.
 * \details A half-edge mesh is comprised of directional half-edges that refer to the next edge in the triangle in
 *          counter-clockwise order. Each half-edge also provides pointers to the vertex at the head of the edge, its
 *          associated triangle face, and its flip edge which represents the same edge in the opposite direction. Using
 *          just these four pointers, one can effectively traverse and modify edges in a triangle mesh.
 */

using vec3_vertex_map = std::unordered_map<glm::vec3, gfx::ProcessingVertex, Vertex::Vec3Hash, Vertex::Vec3Equal>;
using face_set = std::unordered_set<gfx::Face, FaceHash, FaceEqual>;
class HalfEdgeMesh {
public:
  /**
   * \brief Initializes a half-edge mesh.
   * \param mesh An indexed triangle mesh to construct the half-edge mesh from.
   */
  explicit HalfEdgeMesh(const Mesh& mesh);

  struct TriangleInfo {
    std::vector<Face> disappear;
    std::vector<Face> influenceA;
    glm::vec3 posA;
    std::vector<Face> influenceB;
    glm::vec3 posB;
  };
  TriangleInfo collectTriangle(HalfEdgeKey edge);

  /** \brief Gets the mesh vertices by ID. */
  [[nodiscard]] auto& vertices() noexcept { return vertices_; }

  /** \brief Gets the mesh faces by hash value. */
  [[nodiscard]] auto& faces() noexcept { return faces_; }

  /**
   * \brief Performs edge contraction.
   * \details Edge contraction consists of removing an edge from the mesh by merging its two vertices into a single
   *          vertex and updating edges incident to each endpoint to connect to that new vertex.
   * \param edge01 The edge from vertex \c v0 to \c v1 to remove.
   * \param v_new The new vertex to attach edges incident to \c v0 and \c v1 to.
   */
  struct ContractInfo {
    std::vector<HalfEdgeKey> disappear;
    std::vector<Face> newFaces;
  };
  ContractInfo Contract(HalfEdgeKey edge01, glm::vec3 v_new);

  /**
   * \brief Converts the half-edge mesh back to an indexed triangle mesh.
   * \param device The graphics device used to load mesh data into GPU memory.
   * \return An indexed triangle mesh.
   */
  [[nodiscard]] Mesh ToMesh(const Device& device) const;

private:
  vec3_vertex_map vertices_;
  face_set faces_;
  glm::mat4 transform_;
};

}  // namespace gfx

#endif  // SRC_GEOMETRY_INCLUDE_GEOMETRY_HALF_EDGE_MESH_H_
