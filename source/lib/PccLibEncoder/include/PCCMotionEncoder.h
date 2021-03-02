

/*
 * This is the start of all code related to motion estimation
 * in the encoder WIP
 *
 */

#ifndef PCC_APP_MOTION_ESTIMATOR_H
#define PCC_APP_MOTION_ESTIMATOR_H

#include "PCCCommon.h"
#include "PCCPointSet.h"
#include "PCCKdTree.h"
#include <vector>

namespace pcc {
class PCCMotionEncoder {
 public:
  PCCMotionEncoder() : threshold_( 200000 ), neiPoints_( 10 ), dist_( 100 ) {}
  ~PCCMotionEncoder() = default;

  // TODO: method to write as picture?

  uint32_t calculateColorDiff( const PCCPointSet3& currPointCloud, const PCCPointSet3& refPointCloud );
  bool     genMotionData( const PCCPointSet3& currPointCloud, const PCCPointSet3& refPointCloud );
  bool     withinRange( const PCCPoint3D& p1, const PCCPoint3D& p2 );

  bool writeToFile();

 private:
  int              threshold_;
  std::vector<int> matchingPointSet_;
  int              neiPoints_;
  int              dist_;
  PCCNNResult refNei_, currNei_;
};

}  // namespace pcc

#endif
