/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Matej Dudák, Lukáš Lánik
/*****************************************************/

# pragma once

#include <sensor_msgs/PointCloud2.h>

/* SGT */
#include <sgtdv_msgs/Point2DStampedArr.h>
#include "SGT_Utils.h"

class LidarConeDetection
{
public:
  struct Params
  {
    Utils::Range<double> cluster_points;
    Utils::Range<double> intensity;
    Utils::Range<double> x_range;
    Utils::Range<double> y_range;
    Utils::Range<double> z_range;
    double cluster_radius;
    double mean_cone_radius;
  };

public:
  explicit LidarConeDetection() = default;
  ~LidarConeDetection() = default;

  sgtdv_msgs::Point2DStampedArr update(const sensor_msgs::PointCloud2::ConstPtr &msg) const;

  void setParams(const Params& params)
  {
    params_ = params;
  };

private:
  Params params_;
};
