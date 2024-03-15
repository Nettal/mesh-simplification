//
// Created by nettal on 24-3-14.
//

#ifndef MESH_SIMPLIFICATION_HALFEDGEKEY_H
#define MESH_SIMPLIFICATION_HALFEDGEKEY_H

#include <glm/vec3.hpp>
namespace gfx {

class HalfEdgeKey {
  glm::vec3 posTo_;
  glm::vec3 pos_;

public:
  HalfEdgeKey swap() const{
      return HalfEdgeKey(posTo_, pos_);
  }

  glm::vec3 posTo() const { return posTo_; }

  glm::vec3 pos() const { return pos_; }
  HalfEdgeKey(glm::vec3 pos, glm::vec3 toPos) : pos_(pos), posTo_(toPos) {}

  struct HalfEdgeKeyHash {
    std::size_t operator()(const HalfEdgeKey& type) const {
      const auto& pos_ = type.pos_;
      const auto& toPos_ = type.posTo_;
      int64_t result = 1;
      int i = *((int*)(&pos_.x));
      result = 31 * result + i;
      i = *((int*)(&pos_.y));
      result = 31 * result + i;
      i = *((int*)(&pos_.z));
      result = 31 * result + i;
      i = *((int*)(&toPos_.x));
      result = 31 * result + i;
      i = *((int*)(&toPos_.y));
      result = 31 * result + i;
      i = *((int*)(&toPos_.z));
      result = 31 * result + i;
      return result;
    }
  };

  struct HalfEdgeKeyEqual {
    bool operator()(const HalfEdgeKey& lhs, const HalfEdgeKey& rhs) const {
      return lhs.pos_ == rhs.pos_ && lhs.posTo_ == rhs.posTo_;
    }
  };

  bool operator==(const HalfEdgeKey& rhs) const { return HalfEdgeKeyEqual()(rhs, *this); }
};
}  // namespace gfx

#endif  // MESH_SIMPLIFICATION_HALFEDGEKEY_H
