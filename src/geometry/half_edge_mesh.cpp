#include "geometry/half_edge_mesh.h"

#include <algorithm>
#include <cassert>
#include <ranges>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <iostream>

#include "geometry/face.h"
#include "geometry/half_edge.h"
#include "geometry/vertex.h"
#include "graphics/device.h"
#include "graphics/mesh.h"

namespace {

/**
 * \brief Creates a new triangle in the half-edge mesh.
 * \param v0,v1,v2 The triangle vertices in counter-clockwise order.
 * \param edges A mesh half-edges by hash value.
 * \return A triangle face constructed from the vertices \p v0, \p v1, \p v2.
 */
gfx::Face CreateTriangle(gfx::Vertex a, gfx::Vertex b, gfx::Vertex c, gfx::vec3_vertex_map& vertices) {
  gfx::Face face(a, b, c);
  vertices.at(a.position()).addFace(face);
  vertices.at(b.position()).addFace(face);
  vertices.at(c.position()).addFace(face);
  return face;
}

/**
 * \brief Computes a vertex normal by averaging its face normals weighted by surface area.
 * \param v0 The vertex to compute the normal for.
 * \return The weighted vertex normal.
 */
glm::vec3 AverageVertexNormals(const gfx::ProcessingVertex& v0) {
  glm::vec3 normal{0.0f};
  for (const auto& item : v0.faces()) {
    normal += item.normal() * item.area();
  }
  return glm::normalize(normal);
}

}  // namespace

namespace gfx {

HalfEdgeMesh::TriangleInfo HalfEdgeMesh::collectTriangle(HalfEdgeKey edge) {
  ProcessingVertex vertexA = vertices_.at(edge.pos());
  ProcessingVertex vertexB = vertices_.at(edge.posTo());
  std::vector<Face> disappear{};
  std::vector<Face> influenceA{};
  std::vector<Face> influenceB{};
  face_set allTriangle;
  for (const var& item : vertexB.faces()) {
    allTriangle.insert(item);
  }
  for (const var& item : vertexA.faces()) {
    allTriangle.insert(item);
  }
  for (var triangle : allTriangle) {
    ProcessingVertex *a = null, *b = null;
    for (auto vertex : triangle.vertex()) {
      if (vertex.position() == vertexA.pos()) a = &vertexA;
      if (vertex.position() == vertexB.pos()) b = &vertexB;
    }
    if (a != null && b != null) {
      disappear.emplace_back(triangle);
      continue;
    }
    if (a != null) {
      influenceA.emplace_back(triangle);
      continue;
    }
    if (b != null) {
      influenceB.emplace_back(triangle);
      continue;
    }
    assert(0);
  }
  return TriangleInfo(disappear, influenceA, vertexA.pos(), influenceB, vertexB.pos());
}
HalfEdgeMesh::HalfEdgeMesh(const Mesh& mesh)
    : vertices_{mesh.vertices()  //
                | std::views::transform([id = 0u](const auto& mesh_vertex) mutable {
                    auto vertex = ProcessingVertex(mesh_vertex.position);
                    return std::pair{vertex.pos(), vertex};
                  })
                | std::ranges::to<vec3_vertex_map>()},
      faces_{mesh.indices()          //
             | std::views::chunk(3)  //
             | std::views::transform([this, &mesh](const auto& index_group) {
                 auto a = mesh.vertices().at(index_group[0]);
                 auto b = mesh.vertices().at(index_group[1]);
                 auto c = mesh.vertices().at(index_group[2]);
                 auto aa = gfx::Vertex(a.position);
                 auto bb = gfx::Vertex(b.position);
                 auto cc = gfx::Vertex(c.position);
                 auto face012 = CreateTriangle(aa, bb, cc, vertices_);
                 return face012;
               })
             | std::ranges::to<face_set>()},
      transform_{mesh.transform()} {}

template <typename T>
void removeAll(std::vector<T>& source, const std::vector<T>& elementsToRemove) {
  source.erase(std::remove_if(source.begin(),
                              source.end(),
                              [&](const T& element) {
                                for (const auto& item : elementsToRemove) {
                                  if (item == element) {
                                    return true;
                                  }
                                }
                                return false;
                              }),
               source.end());
}
using Vec3f = glm::vec3;
using Vec2f = glm::vec2;
std::vector<HalfEdgeKey> removeTriangle(Face triangle, HalfEdgeMesh& halfEdgeMesh) {
  halfEdgeMesh.faces().erase(triangle);
  for (var vertex : triangle.vertex()) {
    halfEdgeMesh.vertices().at(vertex.position()).removeFace(triangle);
  }
  return triangle.edges();
}

struct TriangleUpdater {
  Face triangle;
  Vec3f oldPos;
  Vec3f newPos;
  HalfEdgeKey edge;
  HalfEdgeMesh& halfEdgeMesh;
  TriangleUpdater(Face triangle, Vec3f oldPos, Vec3f newPos, HalfEdgeMesh& halfEdgeMesh)
      : triangle(triangle), oldPos(oldPos), newPos(newPos), edge({0, 0, 0}, {0, 0, 0}), halfEdgeMesh(halfEdgeMesh) {
    std::vector<HalfEdgeKey> removed;
    std::vector<HalfEdgeKey> triangleEdges = triangle.edges();
    for (var e : triangle.edges()) {
      if (e.pos() == oldPos || e.posTo() == oldPos) {
        // old edge
        removed.emplace_back(e);
      }
    }
    removeAll(triangleEdges, removed);
    assert(triangleEdges.size() == 1);
    // the remain edge
    edge = triangleEdges[0];
  }
  std::vector<HalfEdgeKey> removeOldTriangle() { return removeTriangle(triangle, halfEdgeMesh); }

  Face buildNewTriangle() {
    auto a = edge.pos();
    auto b = edge.posTo();
    auto c = newPos;
    auto f = Face(Vertex(a), Vertex(b), Vertex(c));
    halfEdgeMesh.vertices().at(a).addFace(f);
    halfEdgeMesh.vertices().at(b).addFace(f);
    halfEdgeMesh.vertices().at(c).addFace(f);
    return f;
  }
};

HalfEdgeMesh::ContractInfo HalfEdgeMesh::Contract(HalfEdgeKey edge01, glm::vec3 v_new) {
  assert(!vertices_.contains(v_new));
  assert(vertices_.contains(edge01.pos()));
  assert(vertices_.contains(edge01.posTo()));
  assert(v_new != edge01.pos());
  assert(v_new != edge01.posTo());

  vertices_.emplace(v_new, ProcessingVertex(v_new));
  var triangle = collectTriangle(edge01);
  std::vector<TriangleUpdater> triangle_updaters;
  std::vector<HalfEdgeKey> removed;
  for (const auto& item : triangle.influenceA) triangle_updaters.emplace_back(item, triangle.posA, v_new, *this);
  for (const auto& item : triangle.influenceB) triangle_updaters.emplace_back(item, triangle.posB, v_new, *this);

  for (auto& item : triangle_updaters) {
    for (auto& i : item.removeOldTriangle()) removed.emplace_back(i);
  }

  for (auto& item : triangle.disappear) {
    for (auto& i : removeTriangle(item, *this)) {
      removed.emplace_back(i);
    }
  }

  std::vector<Face> newFace;

  for (auto& item : triangle_updaters) {
    const Face& newTriangle = item.buildNewTriangle();
    faces_.insert(newTriangle);

    newFace.emplace_back(newTriangle);
  }

  vertices_.erase(edge01.pos());
  vertices_.erase(edge01.posTo());
  return {removed, newFace};
}

Mesh HalfEdgeMesh::ToMesh(const Device& device) const {
  std::vector<Mesh::Vertex> vertices;
  vertices.reserve(vertices_.size());

  std::vector<std::uint32_t> indices;
  indices.reserve(3 * faces_.size());

  std::unordered_map<Vec3f, std::uint32_t, Vertex::Vec3Hash, Vertex::Vec3Equal> index_map;
  index_map.reserve(vertices_.size());

  for (std::uint32_t index = 0; auto& v : vertices_ | std::views::values) {
    const Vec3f& pos = v.pos();
    vertices.push_back(Mesh::Vertex{.position = pos, .normal = AverageVertexNormals(v)});
    index_map.emplace(pos, index++);  // map original vertex IDs to new index positions
  }

  for (const auto& face : faces_) {
    indices.push_back(index_map.at(face.a_.position()));
    indices.push_back(index_map.at(face.b_.position()));
    indices.push_back(index_map.at(face.c_.position()));
  }

  return Mesh{device, vertices, indices, transform_};
}

}  // namespace gfx
