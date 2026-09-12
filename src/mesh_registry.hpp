#pragma once
#include "internal.hpp"

namespace p3d {
namespace mesh_detail {
// The native tolerance comparison is not a strict weak ordering. An STL map
// would have undefined requirements here; reproduce its explicit tree walk and
// red/black insertion instead. This registry never changes the source mesh.
template <class Scalar, std::size_t N> class Registry {
    using Value = std::array<Scalar, N>;
    Scalar epsilon;
    struct Node {
        Value value;
        std::size_t child[2]{0, 0}, parent = 0;
        bool red = true;
    };
    std::vector<Node> nodes{{}};
    std::size_t root = 0;
    int compare(const Value &a, const Value &b) const {
        for (unsigned k = 0; k < N; ++k) {
            if (Scalar(a[k] - epsilon) > b[k])
                return 1;
            if (b[k] > Scalar(a[k] + epsilon))
                return -1;
        }
        return 0;
    }
    void rotate(std::size_t x, unsigned side) {
        const auto y = nodes[x].child[1 - side];
        nodes[x].child[1 - side] = nodes[y].child[side];
        if (nodes[y].child[side])
            nodes[nodes[y].child[side]].parent = x;
        const auto p = nodes[x].parent;
        nodes[y].parent = p;
        if (!p)
            root = y;
        else
            nodes[p].child[nodes[p].child[1] == x] = y;
        nodes[y].child[side] = x;
        nodes[x].parent = y;
    }

  public:
    explicit Registry(Scalar tolerance) : epsilon(tolerance) {
        nodes[0].red = false;
    }
    std::size_t insert(Value value) {
        std::size_t p = 0, candidate = 0, x = root;
        unsigned side = 0;
        while (x) {
            p = x;
            side = compare(value, nodes[x].value) > 0;
            if (!side)
                candidate = x;
            x = nodes[x].child[side];
        }
        if (candidate && compare(nodes[candidate].value, value) <= 0)
            return candidate;
        const auto id = nodes.size();
        nodes.push_back(Node{value, {0, 0}, p, true});
        if (!p)
            root = id;
        else
            nodes[p].child[side] = id;
        x = id;
        while (nodes[nodes[x].parent].red) {
            p = nodes[x].parent;
            auto g = nodes[p].parent;
            side = nodes[g].child[1] == p;
            const auto uncle = nodes[g].child[1 - side];
            if (nodes[uncle].red) {
                nodes[p].red = nodes[uncle].red = false;
                nodes[g].red = true;
                x = g;
            } else {
                if (x == nodes[p].child[1 - side]) {
                    x = p;
                    rotate(x, side);
                    p = nodes[x].parent;
                    g = nodes[p].parent;
                }
                nodes[p].red = false;
                nodes[g].red = true;
                rotate(g, 1 - side);
            }
        }
        nodes[root].red = false;
        return id;
    }
    Value at(std::size_t i) const {
        return nodes.at(i).value;
    }
    std::size_t size() const {
        return nodes.size() - 1;
    }
};
inline Point3 unit(Point3 p) {
    const double length = std::sqrt((p[0] * p[0] + p[1] * p[1]) + p[2] * p[2]);
    require(std::isfinite(length), "mesh normal magnitude overflow");
    // GeVec3d::normalize assigns +X when the magnitude is zero.
    if (!(length > 0))
        return {1, 0, 0};
    const double factor = 1 / length;
    for (auto &v : p) {
        v *= factor;
        require(std::isfinite(v), "mesh normal overflow");
    }
    return p;
}
inline Point3 face_normal(Point3 a, Point3 b, Point3 c) {
    for (unsigned k = 0; k < 3; ++k) {
        b[k] -= a[k];
        c[k] -= a[k];
    }
    return unit({b[1] * c[2] - b[2] * c[1], b[2] * c[0] - b[0] * c[2], b[0] * c[1] - b[1] * c[0]});
}
} // namespace mesh_detail
} // namespace p3d
