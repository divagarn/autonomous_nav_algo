#pragma once

#include "datatype/radar_point.hpp"
#include <vector>
#include <opencv2/core.hpp>

namespace radarprocessing
{

    /**
     * @class RadarClusterer
     * @brief Simple DBSCAN-based radar point clusterer that produces 3-D bounding boxes.
     */
    class RadarClusterer
    {
    public:
        /**
         * @param eps        Neighbourhood radius [m]
         * @param min_pts    Minimum cluster size
         * @param padding    Extra padding added to each bbox side [m]
         * @param min_rcs    Minimum RCS filter [dBsm]
         */
        explicit RadarClusterer(float eps = 2.0f,
                                int min_pts = 3,
                                float padding = 0.3f,
                                float min_rcs = -10.0f);

        /**
         * @brief Cluster radar points and compute per-cluster 3-D bounding boxes.
         * @param points Input radar points (radar frame)
         * @return Vector of clusters with bounding boxes
         */
        std::vector<RadarCluster> cluster(const std::vector<RadarPoint> &points) const;

    private:
        float eps_;
        int min_pts_;
        float padding_;
        float min_rcs_;

        std::vector<int> dbscan(const std::vector<RadarPoint> &pts) const;
        RadarBBox3D computeBBox(const std::vector<RadarPoint> &pts) const;
    };

} // namespace radarprocessing
