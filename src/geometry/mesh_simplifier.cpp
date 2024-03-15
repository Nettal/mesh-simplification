#include "geometry/mesh_simplifier.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <format>
#include <iostream>
#include <memory>
#include <print>
#include <queue>
#include <ranges>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "geometry/half_edge.h"
#include "geometry/half_edge_mesh.h"
#include "geometry/vertex.h"
#include "graphics/device.h"
#include "graphics/mesh.h"

namespace {

/**
 * \brief A edge contraction candidate in a half-edge mesh.
 * \details An edge contraction candidate includes all the information necessary to process the next half-edge during
 *          the mesh simplification process.
 */
struct EdgeContraction {
  /**
   * \brief Initializes an edge contraction.
   * \param edge The edge to contract.
   * \param vertex The vertex whose position optimally preserves the original shape of the mesh after edge contraction.
   * \param quadric The error quadric for this edge contraction's vertex.
   * \param cost A metric that quantifies how much the mesh will change after this edge has been contracted.
   * \param valid Indicates if this edge contraction is valid. This is used as a workaround for priority_queue not
   *              providing a method to update an existing entry's priority. As edges are updated in the mesh,
   *              duplicated entries may be inserted in the queue and this property will be used to determine if an
   *              entry refers to the most recent edge update.
   */
  EdgeContraction(const gfx::HalfEdgeKey& edge,
                  const glm::vec3 newPos,
                  const glm::mat4& quadric,
                  const float cost,
                  const bool valid = true)
      : edge{edge}, newPos{newPos}, quadric{quadric}, cost{cost}, valid{valid} {}

  // NOLINTBEGIN(misc-non-private-member-variables-in-classes)
  gfx::HalfEdgeKey edge;
  glm::vec3 newPos;
  glm::mat4 quadric;
  float cost;
  bool valid;
  // NOLINTEND(misc-non-private-member-variables-in-classes)
};

/**
 * \brief Creates an error quadric for a given vertex.
 * \param v0 The vertex to create an error quadric for.
 * \return The error quadric for \p v0.
 */
glm::mat4 CreateErrorQuadric(const gfx::ProcessingVertex& v0) {
  glm::mat4 quadric{0.0f};
  auto pos = v0.pos();
  for (const auto& item : v0.faces()) {
    auto normal = item.normal();
    const glm::vec4 plane{normal, -glm::dot(pos, normal)};
    quadric += glm::outerProduct(plane, plane);
  }
  return quadric;
}

/**
 * \brief Creates an edge contraction candidate.
 * \param edge01 The edge to contract.
 * \param quadrics Error quadrics for each vertex in the half-edge mesh by vertex ID.
 * \return An edge contraction candidate that includes the vertex whose position optimally preserves the original shape
 *         and its associated cost.
 */
EdgeContraction CreateEdgeContraction(const gfx::HalfEdgeKey& edge01, const gfx::mesh::vec3_mat4_map& quadrics) {
  const auto v0 = edge01.pos();

  const auto v1 = edge01.posTo();

  const auto& q0 = quadrics.at(v0);
  const auto& q1 = quadrics.at(v1);
  const auto q01 = q0 + q1;

  if (glm::determinant(q01) == 0.0f) {
    // average the edge vertices if the error quadric is not invertible
    const auto position = (v0 + v1) / 2.0f;
    return {edge01, position, q01, 0.0f};
  }

  auto position = glm::inverse(q01) * glm::vec4{0.0f, 0.0f, 0.0f, 1.0f};
  position /= position.w;

  const auto squared_distance = glm::dot(position, q01 * position);
  return {edge01, position, q01, squared_distance};
}

/**
 * \brief Determines if the removal of an edge will cause the mesh to degenerate.
 * \param edge01 The edge to evaluate.
 * \return \c true if the removal of \p edge01 will produce a non-manifold, otherwise \c false.
 */
bool WillDegenerate(const EdgeContraction& contraction, gfx::vec3_vertex_map& map) {
  if (map.contains(contraction.newPos)) return true;
  return false;
}

}  // namespace

namespace gfx {

Mesh mesh::Simplify(const Device& device, const Mesh& mesh, const float rate) {
  if (rate < 0.0f || rate > 1.0f) {
    throw std::invalid_argument{std::format("Invalid mesh simplification rate: {}", rate)};
  }

  const auto start_time = std::chrono::high_resolution_clock::now();
  HalfEdgeMesh half_edge_mesh{mesh};

  // compute error quadrics for each vertex in the mesh
  vec3_mat4_map quadrics;
  for (const auto& [pos, vertex] : half_edge_mesh.vertices()) {
    quadrics.emplace(pos, CreateErrorQuadric(vertex));
  }

  // use a priority queue to sort edge contraction candidates by the cost of removing each edge
  static constexpr auto kSortByMinCost = [](const auto& lhs, const auto& rhs) { return lhs.cost > rhs.cost; };
  std::priority_queue<EdgeContraction, std::vector<EdgeContraction>, decltype(kSortByMinCost)> edge_contractions{
      kSortByMinCost};

  // this is used to invalidate existing priority queue entries as edges are updated or removed from the mesh
  using half_edge_conn_map = std::unordered_map<gfx::HalfEdgeKey,
                                                EdgeContraction,
                                                gfx::HalfEdgeKey::HalfEdgeKeyHash,
                                                gfx::HalfEdgeKey::HalfEdgeKeyEqual>;
  half_edge_conn_map valid_edges;

  // compute edge contraction candidates for each edge in the mesh
  for (const auto& face : half_edge_mesh.faces()) {
    for (const auto& edge : face.edges()) {
      if (!valid_edges.contains(edge) || valid_edges.contains(edge.swap())) {
        auto edge_contraction = CreateEdgeContraction(edge, quadrics);
        edge_contractions.push(edge_contraction);
        valid_edges.emplace(edge, edge_contraction);
      }
    }
  }

  // stop mesh simplification if the number of triangles has been sufficiently reduced
  const auto initial_face_count = half_edge_mesh.faces().size();
  const auto is_simplified = [&, target_face_count = (1.0f - rate) * static_cast<float>(initial_face_count)] {
    const auto face_count = static_cast<float>(half_edge_mesh.faces().size());
    return edge_contractions.empty() || face_count < target_face_count;
  };

  for (auto next_vertex_id = half_edge_mesh.vertices().size(); !is_simplified();) {
    const auto& edge_contraction = edge_contractions.top();
    edge_contractions.pop();
    const auto& edge01 = edge_contraction.edge;
    if (WillDegenerate(edge_contraction, half_edge_mesh.vertices()) || !valid_edges.contains(edge01)) continue;

    // begin processing the next edge contraction
    valid_edges.erase(edge01);
    valid_edges.erase(edge01.swap());

    const auto& v_new = edge_contraction.newPos;
    quadrics.emplace(v_new, edge_contraction.quadric);

    // remove the edge from the mesh and attach incident edges to the new vertex
    auto info = half_edge_mesh.Contract(edge01, v_new);

    std::cout << "newFace" << info.newFaces.size() << '\n';
    std::cout << "disappear" << info.disappear.size() << '\n';

    for (const auto& item : info.disappear) {
      valid_edges.erase(item);
      valid_edges.erase(item.swap());
    }
    // add new edge contraction candidates for edges affected by the edge contraction
    std::unordered_set<HalfEdgeKey, gfx::HalfEdgeKey::HalfEdgeKeyHash, gfx::HalfEdgeKey::HalfEdgeKeyEqual> newEdge{};

    for (const auto& face : info.newFaces) {
      for (auto item : face.edges()) {
        newEdge.emplace(item);
        newEdge.emplace(item.swap());
        valid_edges.erase(item.swap());
        valid_edges.erase(item);
        quadrics.erase(item.posTo());
        quadrics.erase(item.pos());
      }
    }
    for (const auto& edge : newEdge) {
      quadrics[edge.pos()] = CreateErrorQuadric(half_edge_mesh.vertices().at(edge.pos()));
      quadrics[edge.posTo()] = CreateErrorQuadric(half_edge_mesh.vertices().at(edge.posTo()));
      if (!valid_edges.contains(edge) && !valid_edges.contains(edge.swap())) {
        auto e = CreateEdgeContraction(edge, quadrics);
        valid_edges.emplace(edge, e);
        edge_contractions.emplace(e);
      }
    }
  }

  std::println(std::clog,
               "Mesh simplified from {} to {} triangles in {} seconds",
               initial_face_count,
               half_edge_mesh.faces().size(),
               std::chrono::duration<float>{std::chrono::high_resolution_clock::now() - start_time}.count());

  return half_edge_mesh.ToMesh(device);
}

}  // namespace gfx
