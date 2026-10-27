/**
 * @file ekf_tracker.cpp
 * @author sathishram
 * @brief EKF multi-object tracker — POE lifecycle, CA model, ego-motion
 *        compensation, greedy association, internal DBSCAN.
 *        Mirrors multi_radar_main.py TrackedObject / KalmanFilter logic.
 */

#include "radarprocessing/ekf_tracker.hpp"

#include <cmath>
#include <algorithm>
#include <limits>
#include <numeric>
#include <iostream>
#include <queue>

namespace radarprocessing
{

    // ─────────────────────────────────────────────────────────────────────────────
    // Hungarian (Kuhn-Munkres) assignment — O(nR² · nC), handles nR ≤ nC.
    //   cost[i][j] = Mahalanobis² distance (or kHungInf if outside gate).
    //   Returns assignment[i] = column j matched to row i, -1 = unmatched.
    // ─────────────────────────────────────────────────────────────────────────────
    static constexpr float kHungInf = 1e9f;

    /*std::vector<int> hungarianAssign(const std::vector<std::vector<float>> &cost)
    {
        int nR = static_cast<int>(cost.size());
        if (nR == 0) return {};
        int nC = static_cast<int>(cost[0].size());
        if (nC == 0) return std::vector<int>(nR, -1);

        // Standard O(n²m) Hungarian for nR ≤ nC (cp-algorithms.com "assignment")
        // Rows = tracks, Cols = points.
        // If more tracks than points, transpose, solve, map back.
        bool transposed = (nR > nC);
        const std::vector<std::vector<float>> *cp = &cost;
        std::vector<std::vector<float>> tc_mat;
        if (transposed)
        {
            tc_mat.assign(nC, std::vector<float>(nR));
            for (int i = 0; i < nR; ++i)
                for (int j = 0; j < nC; ++j)
                    tc_mat[j][i] = cost[i][j];
            cp = &tc_mat;
            std::swap(nR, nC);   // nR ≤ nC now
        }

        std::vector<float> u(nR + 1, 0.f), v(nC + 1, 0.f);
        std::vector<int>   p(nC + 1, 0),   way(nC + 1, 0);

        for (int i = 1; i <= nR; ++i)
        {
            p[0] = i;
            int j0 = 0;
            std::vector<float> minv(nC + 1, kHungInf);
            std::vector<bool>  used(nC + 1, false);

            do {
                used[j0] = true;

                int i0 = p[j0];
                if (i0 == 0) break;   // 🚀 critical safety

                int j1 = 0;
                float delta = kHungInf;

                for (int j = 1; j <= nC; ++j)
                {
                    if (!used[j])
                    {
                        float cur = (*cp)[i0 - 1][j - 1] - u[i0] - v[j];

                        if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                        if (minv[j] < delta) { delta = minv[j]; j1 = j; }
                    }
                }

                for (int j = 0; j <= nC; ++j)
                {
                    if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
                    else          minv[j] -= delta;
                }
                j0 = j1;
            } while (j0 != 0 && p[j0] != 0);

            do { int j1 = way[j0]; p[j0] = p[j1]; j0 = j1; } while (j0);
        }

        // Collect result: for each col j, p[j] is the 1-based row assigned to it
        if (!transposed)
        {
            std::vector<int> ans(nR, -1);   // original nR (restored after swap)
            for (int j = 1; j <= nC; ++j)
                if (p[j] > 0 && (*cp)[p[j] - 1][j - 1] < kHungInf)
                    ans[p[j] - 1] = j - 1;
            return ans;
        }
        else
        {
            // transposed: rows were orig cols, swap back
            // nR = original nC, nC = original nR after swap
            // ans[col_orig] = row_orig
            int origR = nC, origC = nR;   // after swap: nR=origC, nC=origR
            std::vector<int> ans(origR, -1);
            for (int j = 1; j <= nC; ++j)
                if (p[j] > 0 && (*cp)[p[j] - 1][j - 1] < kHungInf)
                    ans[j - 1] = p[j] - 1;   // ans[orig_track] = orig_point
            (void)origC;
            return ans;
        }
    }*/

    // ─────────────────────────────────────────────────────────────────────────────
    // Median-based outlier trimming.
    // Computes median (x, y) of the input, removes points whose distance from the
    // median exceeds scale * MAD (median absolute deviation).  Never returns empty.
    // ─────────────────────────────────────────────────────────────────────────────
    std::vector<RadarPoint> trimOutliers(const std::vector<RadarPoint> &pts, float scale)
    {
        int n = static_cast<int>(pts.size());
        if (n <= 2)
            return pts;

        // Compute median X and Y
        std::vector<float> xs(n), ys(n);
        for (int i = 0; i < n; ++i)
        {
            xs[i] = pts[i].x;
            ys[i] = pts[i].y;
        }

        auto med = [](std::vector<float> &v) -> float
        {
            int m = static_cast<int>(v.size()) / 2;
            std::nth_element(v.begin(), v.begin() + m, v.end());
            return v[m];
        };
        float mx = med(xs), my = med(ys);

        // MAD = median of Euclidean distances to (mx, my)
        std::vector<float> devs(n);
        for (int i = 0; i < n; ++i)
            devs[i] = std::hypot(pts[i].x - mx, pts[i].y - my);
        float mad = med(devs);
        float thresh = scale * (mad + 0.1f); // 0.1 m floor

        std::vector<RadarPoint> out;
        out.reserve(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i)
            if (std::hypot(pts[i].x - mx, pts[i].y - my) <= thresh)
                out.push_back(pts[i]);

        return out.empty() ? pts : out; // fallback: never discard everything
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Flat 2-D KD-tree — O(N log N) build, O(log N + k) radius query
    // ─────────────────────────────────────────────────────────────────────────────
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

        void build(const std::vector<RadarPoint> &pts)
        {
            int n = static_cast<int>(pts.size());
            nodes.clear();
            if (n == 0)
                return;
            nodes.reserve(n);
            std::vector<int> idx(n);
            std::iota(idx.begin(), idx.end(), 0);
            buildRec(pts, idx, 0, n, 0);
        }

        void radiusSearch(const std::vector<RadarPoint> &pts,
                          float qx, float qy, float eps,
                          std::vector<int> &out) const
        {
            if (nodes.empty())
                return;
            const float eps2 = eps * eps;
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
            int axis = depth & 1;
            int mid = (lo + hi) / 2;
            std::nth_element(idx.begin() + lo,
                             idx.begin() + mid,
                             idx.begin() + hi,
                             [&](int a, int b)
                             {
                                 return (axis == 0 ? pts[a].x : pts[a].y) <
                                        (axis == 0 ? pts[b].x : pts[b].y);
                             });
            int ni = static_cast<int>(nodes.size());
            float sv = (axis == 0) ? pts[idx[mid]].x : pts[idx[mid]].y;
            nodes.push_back({idx[mid], -1, -1, axis, sv});
            int left = buildRec(pts, idx, lo, mid, depth + 1);
            int right = buildRec(pts, idx, mid + 1, hi, depth + 1);
            nodes[ni].left = left;
            nodes[ni].right = right;
            return ni;
        }
    };

    // ─────────────────────────────────────────────────────────────────────────────
    EkfTracker::EkfTracker(const EkfTrackerConfig &cfg)
        : cfg_(cfg), gating_mode_(gating::GatingMode::ADAPTIVE)
    {
        gating_strategy_ = gating::GatingStrategyFactory::create(gating_mode_, cfg_);
    }

    void EkfTracker::setGatingMode(gating::GatingMode mode)
    {
        gating_mode_ = mode;
        gating_strategy_ = gating::GatingStrategyFactory::create(mode, cfg_);
    }

    void EkfTracker::reset()
    {
        tracks_.clear();
        clusters_.clear();
        next_id_ = 1;
    }

    // Exponential-moving mean/variance update for one scalar signal (RCS).
    // Fast-adapting at birth (alpha ~1 on the first hit, so mean snaps
    // straight to the first sample instead of dragging up from a 0
    // initializer), settling to the same alpha cap used for length/width
    // smoothing. Feeds RadarTrack::rcs_ema_mean, used by the ground/
    // overdrivable classification below.
    static void updateEmaMeanVar(float sample, float alpha, float &mean, float &var)
    {
        float delta = sample - mean;
        mean += alpha * delta;
        var = (1.f - alpha) * (var + alpha * delta * delta);
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Predict: constant-velocity propagation + CWNA Q matrix (matches Python KF)
    // ─────────────────────────────────────────────────────────────────────────────
    void EkfTracker::predict(RadarTrack &t, float dt) const
    {
        t.px += t.vx * dt;
        t.py += t.vy * dt;

        // State transition matrix F (constant-velocity model)
        Eigen::Matrix4f F = Eigen::Matrix4f::Identity();
        F(0, 2) = dt;
        F(1, 3) = dt;

        // CWNA process-noise Q  (Continuous White Noise Acceleration, sigma_a)
        float dt2 = dt * dt * 0.5f; // dt²/2
        float sa2 = cfg_.sigma_a * cfg_.sigma_a;
        Eigen::Matrix4f Q = Eigen::Matrix4f::Zero();
        Q(0, 0) = dt2 * dt2 * sa2;
        Q(0, 2) = dt * dt2 * sa2;
        Q(1, 1) = dt2 * dt2 * sa2;
        Q(1, 3) = dt * dt2 * sa2;
        Q(2, 0) = dt * dt2 * sa2;
        Q(2, 2) = dt * dt * sa2;
        Q(3, 1) = dt * dt2 * sa2;
        Q(3, 3) = dt * dt * sa2;

        t.P = F * t.P * F.transpose() + Q;
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // EKF measurement update — position only, H = [[1,0,0,0],[0,1,0,0]]
    //
    // Measurement noise scales with range: a single point's lateral position
    // error is driven by the sensor's azimuth accuracy (angle_error * range),
    // not a flat constant. DRVEGRD 171 datasheet ("General Performance Data"),
    // Angle / Accuracy: Azimuth: <=0.25 deg, all range modes -- measured at
    // bore sight for a point reflector >25dB SNR. cfg_.meas_noise_pos (1.0m)
    // is close to what that predicts at ~229m, near the Extra-Long-Range
    // mode's 240m max range, so it's kept as the upper cap for far targets;
    // applying it uniformly at close range (as before) made the gate ~13-100x
    // too wide there, letting a track absorb points from separate nearby
    // static reflectors instead of just the one it should be following.
    // Floor (0.2m) matches the existing var_x/var_y floor convention in
    // gating_module.cpp's AdaptiveGating.
    // ─────────────────────────────────────────────────────────────────────────────
    void EkfTracker::updateEKF(RadarTrack &t, float meas_x, float meas_y) const
    {
        // Measurement matrix H = [[1,0,0,0],[0,1,0,0]]
        Eigen::Matrix<float, 2, 4> H = Eigen::Matrix<float, 2, 4>::Zero();
        H(0, 0) = 1.f;
        H(1, 1) = 1.f;

        constexpr float kAzimuthAccuracyRad = 0.25f * (float)M_PI / 180.f;
        float range = std::hypot(meas_x, meas_y);
        float meas_noise_eff = std::clamp(range * kAzimuthAccuracyRad,
                                          0.2f, cfg_.meas_noise_pos);

        float rn = meas_noise_eff * meas_noise_eff;
        //float rn = cfg_.meas_noise_pos * cfg_.meas_noise_pos;

        Eigen::Matrix2f R = Eigen::Matrix2f::Identity() * rn;


        // Innovation covariance S = H*P*H^T + R
        Eigen::Matrix2f S = H * t.P * H.transpose() + R;
        if (std::fabs(S.determinant()) < 1e-12f)
            return;

        // Kalman gain K = P*H^T*S^{-1}  (4×2)
        Eigen::Matrix<float, 4, 2> K = t.P * H.transpose() * S.inverse();

        float inn_x = meas_x - t.px;
        float inn_y = meas_y - t.py;

        t.px += K(0, 0) * inn_x + K(0, 1) * inn_y;
        t.py += K(1, 0) * inn_x + K(1, 1) * inn_y;
        t.vx += K(2, 0) * inn_x + K(2, 1) * inn_y;
        t.vy += K(3, 0) * inn_x + K(3, 1) * inn_y;

        // Joseph form: P = (I-K*H)*P*(I-K*H)^T + K*R*K^T
        Eigen::Matrix4f IKH = Eigen::Matrix4f::Identity() - K * H;
        t.P = IKH * t.P * IKH.transpose() + K * R * K.transpose();

        // Enforce symmetry
        t.P = 0.5f * (t.P + t.P.transpose());
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Mahalanobis-like distance (squared) — uses S = H·P·H^T + R (2×2).
    // H = [[1,0,0,0],[0,1,0,0]]  →  S[0,0]=P[0,0]+R, S[1,1]=P[1,1]+R,
    //                                S[0,1]=P[0,1],    S[1,0]=P[1,0].
    // Returns d² = Δx^T · S^{-1} · Δx.
    // Compare against cfg_.gate_maha_dynamic or gate_maha_static.
    // ─────────────────────────────────────────────────────────────────────────────
    float EkfTracker::mahalanobisDistance(const RadarTrack &t,
                                          float mx, float my) const
    {
        float r2 = cfg_.meas_noise_pos * cfg_.meas_noise_pos;
        float S00 = t.P(0, 0) + r2;
        float S01 = t.P(0, 1);
        float S10 = t.P(1, 0);
        float S11 = t.P(1, 1) + r2;

        float det = S00 * S11 - S01 * S10;
        if (std::fabs(det) < 1e-10f)
            return 1e9f; // singular — reject

        float inv = 1.f / det;
        float dx = mx - t.px;
        float dy = my - t.py;

        // d² = [dx,dy] · S^{-1} · [dx,dy]^T
        float Si00 = S11 * inv;
        float Si01 = -S01 * inv;
        float Si10 = -S10 * inv;
        float Si11 = S00 * inv;

        return dx * (Si00 * dx + Si01 * dy) + dy * (Si10 * dx + Si11 * dy);
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // DBSCAN on XY plane — O(N log N) with KD-tree radius queries.
    // min_pts = 1: every point is a core point (any singleton is its own cluster).
    // ─────────────────────────────────────────────────────────────────────────────
    inline float EkfTracker::adaptiveEps(const RadarPoint &p) const
    {
        // float r = std::hypot(p.x, p.y);

        // Option 1: smooth scaling (recommended)
        float eps = 1.0f; // 0.02f * r;   // 2% of range

        // Clamp to safe bounds
        return std::clamp(eps, 0.5f, 1.5f);
    }

    std::vector<int> EkfTracker::dbscanXY(const std::vector<RadarPoint> &pts) const
    {
        int n = static_cast<int>(pts.size());
        std::vector<int> labels(n, -1);
        if (n == 0)
            return labels;

        // Build KD-tree once per call — O(N log N)
        KdTree2D tree;
        tree.build(pts);

        int cur_label = 0;
        std::vector<int> nb;

        for (int i = 0; i < n; ++i)
        {
            if (labels[i] >= 0)
                continue; // already labelled

            // Reject suspicious high-speed close-lane points
            if (pts[i].radial_speed < -25.0f &&
                std::abs(pts[i].y) < 2.2f &&
                pts[i].x < 200.0f)
            {
                continue;
            }

            labels[i] = cur_label;

            // BFS flood-fill: start from i, expand through all reachable points
            std::queue<int> bfs;
            bfs.push(i);

            while (!bfs.empty())
            {
                int cur = bfs.front();
                bfs.pop();

                nb.clear();
                float eps = adaptiveEps(pts[cur]);
                tree.radiusSearch(pts, pts[cur].x, pts[cur].y,
                                  eps, nb);

                // Every point is a core point (min_pts = 1), so expand all unvisited
                for (int j : nb)
                {
                    if (labels[j] >= 0)
                        continue; // skip already-assigned
                    // 🔥 Velocity consistency check
                    // if (std::abs(pts[j].radial_speed - pts[cur].radial_speed) > 2.0f) continue;
                     // Reject suspicious points
                    if (pts[j].radial_speed < -25.0f &&
                        std::abs(pts[j].y) < 2.2f &&
                        pts[j].x < 200.0f)
                    {
                        continue;
                    }
                    labels[j] = cur_label;
                    bfs.push(j);
                }
            }
            ++cur_label;
        }
        return labels;
    }

    // ─────────────────────────────────────────────────────────────────────────────
    RadarBBox3D EkfTracker::computeBBoxFromPoints(const std::vector<RadarPoint> &pts)
    {
        RadarBBox3D bb;
        bb.x_min = bb.y_min = bb.z_min = 1e9f;
        bb.x_max = bb.y_max = bb.z_max = -1e9f;
        for (auto &p : pts)
        {
            bb.x_min = std::min(bb.x_min, p.x);
            bb.x_max = std::max(bb.x_max, p.x);
            bb.y_min = std::min(bb.y_min, p.y);
            bb.y_max = std::max(bb.y_max, p.y);
            bb.z_min = std::min(bb.z_min, p.z);
            bb.z_max = std::max(bb.z_max, p.z);
        }
        return bb;
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Main update — new pipeline:
    //   Points → Point-to-Track Mahalanobis association (greedy)
    //          → Cluster unassigned points (DBSCAN)
    //          → Spawn new tracks from unmatched clusters
    // ─────────────────────────────────────────────────────────────────────────────
    std::vector<RadarTrack> EkfTracker::update(
        const std::vector<RadarPoint> &points,
        float ego_speed,
        float dt_s)
    {
        if (!cfg_.enabled)
            return {};

        const float dt = std::max(dt_s, 1e-3f);
        const int nPt = static_cast<int>(points.size());
        const int nT = static_cast<int>(tracks_.size());

        // ── 1. Predict all tracks ─────────────────────────────────────────────────
        for (auto &t : tracks_)
        {
            t.py_pre_predict = t.py; // see RadarTrack::py_pre_predict's doc comment
            predict(t, dt);
            t.age++;
        }

        // ── 2. Pre-compute per-point ego-compensated radial speed ─────────────────
        std::vector<float> pt_spd(nPt);
        for (int i = 0; i < nPt; ++i)
        {
            // float az = std::atan2(points[i].y, points[i].x);
            pt_spd[i] = points[i].radial_speed;
        }

        // Build KD-tree once
        KdTree2D tree;
        tree.build(points);

        // Output
        std::vector<bool> pt_used(nPt, false);
        std::vector<std::vector<int>> track_pts(nT);

        // Temporary buffer
        std::vector<int> candidates;

        // Greedy: iterate points (better than tracks for radar sparsity)
        for (int i = 0; i < nPt; ++i)
        {
            candidates.clear();

            // Find the best-scoring track for this point via the gating strategy
            float best_score = kHungInf;
            int best_t = -1;

            for (int j = 0; j < nT; ++j)
            {
                auto &t = tracks_[j];
                float score;

                // Use modular gating strategy
                if (gating_strategy_->evaluate(t, points[i], pt_spd[i], ego_speed, score))
                {
                    if (score < best_score)
                    {
                        best_score = score;
                        best_t = j;
                    }
                }
            }

            // Assign point → best track
            if (best_t >= 0)
            {
                track_pts[best_t].push_back(i);
                pt_used[i] = true;
            }
        }

        // ── 4. Update tracks with their associated points ─────────────────────────
        for (int j = 0; j < nT; ++j)
        {
            auto &t = tracks_[j];
            const auto &plist = track_pts[j];

            if (plist.empty())
            {
                t.missed_frames++;
                // Track is already predicted at the beginning of update()
                // So we just need to decay POE and continue
                // Exponential decay based on track status
                if (t.confirmed)
                {
                    // Confirmed tracks decay slower (5% per frame)
                    t.poe = t.poe * 0.95f;
                }
                else
                {
                    // Tentative tracks decay faster (15% per frame)
                    t.poe = t.poe * 0.85f;
                }
                t.poe = std::max(0.f, t.poe);

                if (t.poe < cfg_.poe_unconfirm)
                {
                    t.confirmed = false;
                }
                continue;
            }

            // Collect raw points then apply median-based outlier trimming
            std::vector<RadarPoint> raw_pts;
            raw_pts.reserve(plist.size());
            for (int pi : plist)
                raw_pts.push_back(points[pi]);

            // std::vector<RadarPoint> trim_pts = trimOutliers(raw_pts, cfg_.dbscan_outlier_scale);

            // Aggregate trimmed points
            float sx = 0, sy = 0, sz = 0, ss = 0, sr = 0, sn = 0, srange = 0;
            float saz = 0, sel = 0;
            int naz = 0, nel = 0;
            float xmin = 1e9f, xmax = -1e9f, ymin = 1e9f, ymax = -1e9f;
            for (size_t k = 0; k < raw_pts.size(); ++k)
            // for (size_t k = 0; k < trim_pts.size(); ++k)
            {
                const auto &p = raw_pts[k];
                sx += p.x;
                sy += p.y;
                sz += p.z;

                // Find matching speed (look up by position equality)
                // Use pt_spd of the original index if available; fallback to raw computation
                float az = std::atan2(p.y, p.x);
                ss += p.radial_speed - ego_speed * std::cos(az);
                sr += p.rcs;
                sn += p.snr;
                srange += p.range;
                if (!std::isnan(p.azimuth))
                {
                    saz += p.azimuth;
                    ++naz;
                }
                if (!std::isnan(p.elevation))
                {
                    sel += p.elevation;
                    ++nel;
                }
                xmin = std::min(xmin, p.x);
                xmax = std::max(xmax, p.x);
                ymin = std::min(ymin, p.y);
                ymax = std::max(ymax, p.y);
            }
            float fn = static_cast<float>(raw_pts.size());
            float cx = sx / fn;
            float cy = sy / fn;
            float cz = sz / fn;
            float avg_speed = ss / fn;
            float length = 0.0f;
            float width = 0.0f;

            // Dynamic/stationary classification, using last frame's vx/vy
            // (this frame's updateEKF() correction hasn't run yet) -- feeds
            // the static-track lateral rate limit right below.
            t.is_dynamic = (std::hypot(t.vx, t.vy) > cfg_.dynamic_speed_thresh);

            // EKF position update
            updateEKF(t, cx, cy);
            t.pz = cz; // direct assignment for height (no EKF update)

            // Static-track lateral rate limit -- see
            // cfg_.static_lateral_rate_limit's doc comment in
            // tracker_config.hpp. Bounds this frame's TOTAL position
            // movement -- predict()'s own step (t.vy*dt, already run this
            // frame in the "predict all tracks" loop above) PLUS
            // updateEKF()'s own correction -- against t.py_pre_predict
            // (captured before predict() ran this frame). Clamping only the
            // update's correction on top of an already-moved py would let
            // both steps independently hit the cap in the same frame,
            // silently doubling the achieved rate. t.vy is also clamped so
            // drift doesn't resume next frame via predict() alone. px/vx
            // are never touched -- approach speed is expected and normal.
            if (!t.is_dynamic)
            {
                float max_lat_step = cfg_.static_lateral_rate_limit * dt;
                float dy = std::clamp(t.py - t.py_pre_predict, -max_lat_step, max_lat_step);
                t.py = t.py_pre_predict + dy;
                t.vy = std::clamp(t.vy, -cfg_.static_lateral_rate_limit,
                                  cfg_.static_lateral_rate_limit);
            }
            /*  t.hits = 0;
               // ── Velocity: displacement-based blend + radial correction ────────────
               // Displacement velocity (only on frames after first hit)
               if (t.hits > 0)
               {
                   float vx_meas = (cx - t.prev_px) / dt;
                   float vy_meas = (cy - t.prev_py) / dt;
                   const float beta = 0.3f;
                   // Clamp velocities to realistic values (max 50 m/s = 180 km/h)
                   vx_meas = std::clamp(vx_meas, -50.0f, 50.0f);
                   vy_meas = std::clamp(vy_meas, -50.0f, 50.0f);

                   t.vx = (1.f - beta) * t.vx + beta * vx_meas;
                   t.vy = (1.f - beta) * t.vy + beta * vy_meas;

                   // Final clamp to prevent divergence
                   t.vx = std::clamp(t.vx, -50.0f, 50.0f);
                   t.vy = std::clamp(t.vy, -50.0f, 50.0f);
               }*/
            t.prev_px = cx;
            t.prev_py = cy;
            // t.hits++;

            // Orientation update
            float speed_sq = t.vx * t.vx + t.vy * t.vy;

            if (speed_sq > 1e-4f)
                t.yaw = std::atan2(t.vy, t.vx);
            else
                t.yaw = std::atan2(t.py, t.px); // fallback
            // Radial speed velocity correction (Python gain = 0.2)
            // float px2 = t.px, py2 = t.py;
            // float r2  = std::hypot(px2, py2) + 1e-6f;
            // float pred_rad = (t.vx * px2 + t.vy * py2) / r2;
            // float err      = avg_speed - pred_rad;
            // const float vgain = 0.2f;
            // t.vx += vgain * err * px2 / r2;
            // t.vy += vgain * err * py2 / r2;
            // ─────────────────────────────────────────────────────────────────────

            // Rotate points into object frame
            float cos_t = std::cos(t.yaw);
            float sin_t = std::sin(t.yaw);

            float min_long = 1e9f, max_long = -1e9f;
            float min_lat = 1e9f, max_lat = -1e9f;

            for (const auto &p : raw_pts)
            {
                float dx = p.x - t.px;
                float dy = p.y - t.py;

                float d_long = cos_t * dx + sin_t * dy;
                float d_lat = -sin_t * dx + cos_t * dy;

                min_long = std::min(min_long, d_long);
                max_long = std::max(max_long, d_long);
                min_lat = std::min(min_lat, d_lat);
                max_lat = std::max(max_lat, d_lat);
            }

            // Object-frame dimensions
            length = std::clamp(max_long - min_long, 0.5f, 10.f);
            width = std::clamp(max_lat - min_lat, 0.5f, 5.f);

            // Dimension smoothing (exponential, faster at birth)
            float alpha = std::min(1.f / (t.hits + 1.f), 0.3f);
            t.length = (1.f - alpha) * t.length + alpha * length;
            t.width = (1.f - alpha) * t.width + alpha * width;

            // Signal stats
            t.avg_rcs = sr / fn;
            t.avg_snr = sn / fn;

            // Ground/overdrivable classification (see
            // EkfTrackerConfig::ground_z_threshold's doc comment)
            t.z_ema_mean = (1.f - alpha) * t.z_ema_mean + alpha * t.pz;
            updateEmaMeanVar(t.avg_rcs, alpha, t.rcs_ema_mean, t.rcs_ema_var);
            t.is_ground_reflection = (t.z_ema_mean < cfg_.ground_z_threshold) &&
                                     (t.rcs_ema_mean < cfg_.ground_rcs_max);

            t.avg_range = srange / fn;
            t.avg_speed = std::fabs(avg_speed);
            t.radial_speed = avg_speed;
            t.avg_azimuth = naz > 0 ? saz / static_cast<float>(naz) : std::numeric_limits<float>::quiet_NaN();
            t.avg_elevation = nel > 0 ? sel / static_cast<float>(nel) : std::numeric_limits<float>::quiet_NaN();
            t.n_pts = static_cast<int>(raw_pts.size());
            t.bbox.x_min = xmin;
            t.bbox.x_max = xmax;
            t.bbox.y_min = ymin;
            t.bbox.y_max = ymax;
            t.bbox.z_min = t.bbox.z_max = 0.f;

            // POE lifecycle
            t.hits++;
            t.missed_frames = 0;
            t.poe = std::min(1.f, t.poe + cfg_.poe_gain);
            if (t.poe >= cfg_.poe_confirm && t.age > 6)
                t.confirmed = true;
            if (t.poe < cfg_.poe_unconfirm)
                t.confirmed = false;

            // Full suppression: a ground/overdrivable-classified track must
            // never be confirmed, regardless of POE. It's still tracked
            // internally (age/hits/POE keep accumulating in case it's later
            // reclassified), but every confirmed-only consumer now skips
            // it automatically -- MioSelector's own confirmed check,
            // aeb_process_frame_mois()'s out_tracks[]/Top5-fallback export
            // (both already gate on t.confirmed), and therefore the
            // on-board radar visualization GUI (radar_visualization.c's
            // drawAEBRadarRightPanel()), which only draws tracks present in
            // that confirmed-only out_tracks[] array.
            if (t.is_ground_reflection)
                t.confirmed = false;
        }

        // ── 5. Build visualization clusters ──────────────────────────────────────
        // 5a. One cluster per matched track (its associated raw points)
        clusters_.clear();
        for (int j = 0; j < nT; ++j)
        {
            const auto &plist = track_pts[j];
            if (plist.empty())
                continue;
            RadarCluster cl;
            cl.id = static_cast<int>(tracks_[j].id);
            cl.points.reserve(plist.size());
            for (int pi : plist)
                cl.points.push_back(points[pi]);
            cl.bbox = computeBBoxFromPoints(cl.points);
            clusters_.push_back(std::move(cl));
        }

        // 5c. DBSCAN on unassigned points → spawn new tracks, append to clusters_
        std::vector<RadarPoint> unassigned;
        unassigned.reserve(static_cast<size_t>(nPt));
        for (int i = 0; i < nPt; ++i)
            if (!pt_used[i])
                unassigned.push_back(points[i]);

        this->createdTracksCount = 0;

        if (!unassigned.empty())
        {
            auto labels = dbscanXY(unassigned);
            int max_label = *std::max_element(labels.begin(), labels.end());

            if (max_label >= 0)
            {
                int base_id = static_cast<int>(clusters_.size());
                std::vector<RadarCluster> new_cls(static_cast<size_t>(max_label + 1));
                for (int i = 0; i <= max_label; ++i)
                    new_cls[i].id = base_id + i;
                for (size_t i = 0; i < unassigned.size(); ++i)
                    if (labels[i] >= 0)
                        new_cls[labels[i]].points.push_back(unassigned[i]);

                new_cls.erase(std::remove_if(new_cls.begin(), new_cls.end(),
                                             [](const RadarCluster &c)
                                             { return c.points.empty(); }),
                              new_cls.end());
                for (auto &cl : new_cls)
                    cl.bbox = computeBBoxFromPoints(cl.points);

                // Spawn new tracks from unmatched clusters
                for (auto &cl : new_cls)
                {
                    float sx = 0, sy = 0, sz = 0, ss = 0, sr = 0, sn = 0, srange = 0;
                    float saz = 0, sel = 0;
                    int naz = 0, nel = 0;
                    float xmin = 1e9f, xmax = -1e9f, ymin = 1e9f, ymax = -1e9f;
                    for (auto &p : cl.points)
                    {
                        sx += p.x;
                        sy += p.y;
                        sz += p.z;
                        // float az = std::atan2(p.y, p.x);
                        ss += p.radial_speed;
                        sr += p.rcs;
                        sn += p.snr;
                        srange += p.range;
                        if (!std::isnan(p.azimuth))
                        {
                            saz += p.azimuth;
                            ++naz;
                        }
                        if (!std::isnan(p.elevation))
                        {
                            sel += p.elevation;
                            ++nel;
                        }
                        xmin = std::min(xmin, p.x);
                        xmax = std::max(xmax, p.x);
                        ymin = std::min(ymin, p.y);
                        ymax = std::max(ymax, p.y);
                    }
                    float n = static_cast<float>(cl.points.size());
                    this->createdTracksCount++;
                    RadarTrack t;
                    t.id = newId();
                    t.px = sx / n;
                    t.py = sy / n;
                    t.pz = sz / n;
                    t.radial_speed = ss / n;
                    float r = std::hypot(t.px, t.py) + 1e-6f;
                    float vr = t.radial_speed;
                    t.vx = vr * t.px / r;
                    t.vy = vr * t.py / r;
                    t.is_dynamic = (std::hypot(t.vx, t.vy) > cfg_.dynamic_speed_thresh);
                    // Same static lateral rate limit as the matched-update
                    // path (see cfg_.static_lateral_rate_limit's doc
                    // comment) -- without this, a track born with a large
                    // raw vy (this radial-projection estimate has no
                    // history to smooth yet) gets one uncapped predict()
                    // jump on its very next frame.
                    if (!t.is_dynamic)
                    {
                        t.vy = std::clamp(t.vy, -cfg_.static_lateral_rate_limit,
                                          cfg_.static_lateral_rate_limit);
                    }
                    t.yaw = std::atan2(t.vy, t.vx);
                    t.prev_px = sx / n; // init prev = current so first disp = 0
                    t.prev_py = sy / n;
                    t.poe = cfg_.poe_init;
                    t.hits = 1;
                    t.missed_frames = 0;
                    t.age = 1;
                    t.confirmed = false;
                    t.length = std::clamp(xmax - xmin, 0.2f, 10.f);
                    t.width = std::clamp(ymax - ymin, 0.2f, 5.f);
                    t.avg_rcs = sr / n;
                    t.avg_snr = sn / n;
                    // Seed ground/overdrivable classification from this
                    // first observation (see ground_z_threshold/
                    // ground_rcs_max's doc comments).
                    t.z_ema_mean = t.pz;
                    t.rcs_ema_mean = t.avg_rcs;
                    t.rcs_ema_var = 0.f;
                    t.is_ground_reflection = (t.z_ema_mean < cfg_.ground_z_threshold) &&
                                             (t.rcs_ema_mean < cfg_.ground_rcs_max);
                    t.avg_range = srange / n;
                    t.avg_speed = std::fabs(ss / n);
                    t.avg_azimuth = naz > 0 ? saz / static_cast<float>(naz) : std::numeric_limits<float>::quiet_NaN();
                    t.avg_elevation = nel > 0 ? sel / static_cast<float>(nel) : std::numeric_limits<float>::quiet_NaN();
                    t.n_pts = static_cast<int>(cl.points.size());
                    t.bbox = cl.bbox;

                    // Initial covariance — match Python "normal mode"
                    t.P = Eigen::Matrix4f::Zero();
                    t.P(0, 0) = 2.0f; // position variance x
                    t.P(1, 1) = 2.0f; // position variance y
                    t.P(2, 2) = 5.0f; // velocity variance x
                    t.P(3, 3) = 5.0f; // velocity variance y

                    tracks_.push_back(t);
                }

                // Append new (unassigned) clusters to the visualization list
                for (auto &cl : new_cls)
                    clusters_.push_back(std::move(cl));
            }
        }

        this->deletedTracksCount = 0;
        // ── 6. Delete stale / lost tracks ────────────────────────────────────────
        tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                     [&](const RadarTrack &t)
                                     {
                                         if (t.poe <= cfg_.poe_delete 
                                          || (t.missed_frames > cfg_.max_missed_frames  && t.age <= 20 )
                                          || (t.missed_frames > 15 && t.age > 20 ))
                                         {
                                             this->deletedTracksCount++;
                                             // return false;
                                         }
                                         return t.poe <= cfg_.poe_delete ||
                                                (t.missed_frames > cfg_.max_missed_frames && t.age <= 20)
                                                || (t.missed_frames > 15 && t.age > 20);
                                     }),
                      tracks_.end());

        // this->deletedTracksCount += static_cast<int>(std::count_if(tracks_.begin(), tracks_.end(),
        //                                                            [&](const RadarTrack &t)
        //                                                            {
        //                                                                return t.poe <= cfg_.poe_delete ||
        //                                                                       t.missed_frames > cfg_.max_missed_frames;
        //                                                            }));
        /*printf("=============================================================\n");
        // ── 7. Debugging output ─────────────────────────────────────────────────
        for (const auto &t : tracks_)
        {
            printf("Track %d: px = %f, py = %f, vx = %f, vy = %f, poe = %f, missed_frames = %d\n",
                   t.id, t.px, t.py, t.vx, t.vy, t.poe, t.missed_frames);
        }
        */
        if (tracks_.size() > 0 ){
            mirror_detector_.classify(tracks_, ego_speed);
        }
        return tracks_;
    }

    EkfTracker::TrackerState EkfTracker::saveState() const
    {
        TrackerState state;
        state.tracks = tracks_;
        state.next_id = next_id_;
        state.clusters = clusters_;
        return state;
    }

    void EkfTracker::restoreState(const TrackerState &state)
    {
        tracks_ = state.tracks;
        next_id_ = state.next_id;
        clusters_ = state.clusters;

        // Safety: next_id_ must be strictly greater than every restored track ID.
        // This guards against stale/zero next_id values that were never properly saved.
        for (const auto &t : tracks_)
            if (t.id >= next_id_)
                next_id_ = t.id + 1;
    }
} // namespace radarprocessing
