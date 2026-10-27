#include "radarprocessing/radar_clusterer.hpp"

#include <cmath>
#include <algorithm>
#include <numeric>
#include <queue>
#include <limits>

namespace radarprocessing
{

    // ─────────────────────────────────────────────────────────────────────────────
    // Flat 2-D KD-tree — O(N log N) build, O(log N + k) radius query
    // ─────────────────────────────────────────────────────────────────────────────
    namespace
    {

        struct KdTree2D
        {
            struct Node
            {
                int pt_idx;  ///< index into original pts array
                int left;    ///< left  child node index (-1 = none)
                int right;   ///< right child node index (-1 = none)
                int axis;    ///< split axis: 0 = X, 1 = Y
                float split; ///< split plane coordinate
            };

            std::vector<Node> nodes;

            /// Build the tree from a RadarPoint vector (XY only).
            void build(const std::vector<RadarPoint> &pts)
            {
                int n = static_cast<int>(pts.size());
                nodes.clear();
                if (n == 0)
                    return;
                nodes.reserve(n); // exactly one node per point
                std::vector<int> idx(n);
                std::iota(idx.begin(), idx.end(), 0);
                buildRec(pts, idx, 0, n, 0);
            }

            /// Fill `out` with indices of all points within eps of (qx, qy).
            void radiusSearch(const std::vector<RadarPoint> &pts,
                              float qx, float qy, float eps,
                              std::vector<int> &out) const
            {
                if (nodes.empty())
                    return;
                const float eps2 = eps * eps;

                // Iterative DFS — depth never exceeds ~20 for N < 1 000 000
                int stack[64];
                int top = 0;
                stack[top++] = 0;
                while (top > 0)
                {
                    int ni = stack[--top];
                    const Node &nd = nodes[ni];
                    float dx = pts[nd.pt_idx].x - qx;
                    float dy = pts[nd.pt_idx].y - qy;
                    if (dx * dx + dy * dy <= eps2)
                        out.push_back(nd.pt_idx);

                    float diff = (nd.axis == 0) ? (qx - nd.split)
                                                : (qy - nd.split);
                    int near = (diff < 0.f) ? nd.left : nd.right;
                    int far = (diff < 0.f) ? nd.right : nd.left;
                    if (near >= 0)
                        stack[top++] = near;
                    if (far >= 0 && diff * diff <= eps2)
                        stack[top++] = far;
                }
            }

        private:
            int buildRec(const std::vector<RadarPoint> &pts,
                         std::vector<int> &idx, int lo, int hi, int depth)
            {
                if (lo >= hi)
                    return -1;
                int axis = depth & 1; // 0 = X, 1 = Y
                int mid = (lo + hi) / 2;

                // Partial sort: put the median point at position `mid`
                std::nth_element(idx.begin() + lo,
                                 idx.begin() + mid,
                                 idx.begin() + hi,
                                 [&](int a, int b)
                                 {
                                     return (axis == 0 ? pts[a].x : pts[a].y) <
                                            (axis == 0 ? pts[b].x : pts[b].y);
                                 });

                int ni = static_cast<int>(nodes.size());
                float split_val = (axis == 0) ? pts[idx[mid]].x : pts[idx[mid]].y;
                nodes.push_back({idx[mid], -1, -1, axis, split_val});

                int left = buildRec(pts, idx, lo, mid, depth + 1);
                int right = buildRec(pts, idx, mid + 1, hi, depth + 1);
                // Re-access by index (safe after any vector reallocation)
                nodes[ni].left = left;
                nodes[ni].right = right;
                return ni;
            }
        };

    } // anonymous namespace

} // namespace radarprocessing

// ─────────────────────────────────────────────────────────────────────────────
// RadarBBox3D helpers  (global struct — defined outside radarprocessing)
// ─────────────────────────────────────────────────────────────────────────────
std::vector<cv::Point3f> RadarBBox3D::corners() const
{
    return {
        {x_min, y_min, z_min},
        {x_max, y_min, z_min},
        {x_max, y_max, z_min},
        {x_min, y_max, z_min},
        {x_min, y_min, z_max},
        {x_max, y_min, z_max},
        {x_max, y_max, z_max},
        {x_min, y_max, z_max},
    };
}

namespace radarprocessing
{

    // ─────────────────────────────────────────────────────────────────────────────
    // RadarClusterer
    // ─────────────────────────────────────────────────────────────────────────────
    RadarClusterer::RadarClusterer(float eps, int min_pts, float padding, float min_rcs)
        : eps_(eps), min_pts_(min_pts), padding_(padding), min_rcs_(min_rcs)
    {
    }

    // DBSCAN (XY plane) using KD-tree radius queries — O(N log N) average
    std::vector<int> RadarClusterer::dbscan(const std::vector<RadarPoint> &pts) const
    {
        const int N = static_cast<int>(pts.size());
        std::vector<int> labels(N, -1); // -1 = unvisited / noise
        if (N == 0)
            return labels;

        // Build KD-tree once
        KdTree2D tree;
        tree.build(pts);

        int cluster_id = 0;
        std::vector<int> nb;

        for (int i = 0; i < N; ++i)
        {
            if (labels[i] != -1)
                continue; // already assigned

            // Query radius (result includes i itself)
            nb.clear();
            tree.radiusSearch(pts, pts[i].x, pts[i].y, eps_, nb);

            // nb.size() includes the point itself, so core condition: >= min_pts_
            if ((int)nb.size() < min_pts_)
                continue; // noise (may be re-assigned later as border point)

            labels[i] = cluster_id;
            std::queue<int> seeds;
            for (int ni : nb)
                if (ni != i)
                    seeds.push(ni);

            while (!seeds.empty())
            {
                int q = seeds.front();
                seeds.pop();

                if (labels[q] != -1 && labels[q] != cluster_id)
                    continue; // already in a different cluster

                labels[q] = cluster_id;

                nb.clear();
                tree.radiusSearch(pts, pts[q].x, pts[q].y, eps_, nb);
                if ((int)nb.size() >= min_pts_) // q is also a core point
                    for (int ni : nb)
                        if (labels[ni] == -1)
                            seeds.push(ni);
            }
            ++cluster_id;
        }
        return labels;
    }

    RadarBBox3D RadarClusterer::computeBBox(const std::vector<RadarPoint> &pts) const
    {
        RadarBBox3D bb;
        bb.x_min = bb.y_min = bb.z_min = std::numeric_limits<float>::max();
        bb.x_max = bb.y_max = bb.z_max = -std::numeric_limits<float>::max();

        for (auto &p : pts)
        {
            bb.x_min = std::min(bb.x_min, p.x);
            bb.x_max = std::max(bb.x_max, p.x);
            bb.y_min = std::min(bb.y_min, p.y);
            bb.y_max = std::max(bb.y_max, p.y);
            bb.z_min = std::min(bb.z_min, p.z);
            bb.z_max = std::max(bb.z_max, p.z);
        }

        bb.x_min -= padding_;
        bb.x_max += padding_;
        bb.y_min -= padding_;
        bb.y_max += padding_;
        bb.z_min -= padding_;
        bb.z_max += padding_;

        return bb;
    }

    std::vector<RadarCluster> RadarClusterer::cluster(const std::vector<RadarPoint> &points) const
    {
        // RCS filter
        std::vector<RadarPoint> filtered;
        filtered.reserve(points.size());
        for (auto &p : points)
            if (p.rcs >= min_rcs_)
                filtered.push_back(p);

        if (filtered.empty())
            return {};

        auto labels = dbscan(filtered);

        // Group by cluster label
        int max_label = *std::max_element(labels.begin(), labels.end());
        std::vector<RadarCluster> clusters;
        clusters.resize(max_label + 1);

        for (int i = 0; i <= max_label; ++i)
            clusters[i].id = i;

        for (size_t i = 0; i < filtered.size(); ++i)
            if (labels[i] >= 0)
                clusters[labels[i]].points.push_back(filtered[i]);

        // Remove empty (shouldn't happen) and compute bboxes
        std::vector<RadarCluster> result;
        for (auto &c : clusters)
        {
            if (c.points.empty())
                continue;
            c.bbox = computeBBox(c.points);
            result.push_back(std::move(c));
        }

        return result;
    }

} // namespace radarprocessing
