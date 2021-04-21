

/*
 * This is the start of all code related to motion estimation
 * in the encoder WIP
 *
 */

#ifndef PCC_APP_MOTION_ESTIMATOR_H
#define PCC_APP_MOTION_ESTIMATOR_H

#include "PCCCommon.h"
#include "PCCPointSet.h"
#include "PCCImage.h"
#include "PCCKdTree.h"
#include <vector>
#include <unordered_set>

namespace pcc {
class PCCMotionEncoder {
 public:
  PCCMotionEncoder() : threshold_( 2000 ), neiPoints_( 5 ), dist_( 50 ) {}
  ~PCCMotionEncoder() = default;

  // TODO: method to write as picture?
  // return reconstructed point cloud
  // dump in memory store into file
  // read from file and write as picture

  bool buildMatchingPoints( const PCCPointSet3& currPointCloud, const PCCPointSet3& refPointCloud );
  bool buildMatchingPointsFlipped( const PCCPointSet3& currPointCloud, const PCCPointSet3& refPointCloud ); 

  void debugPos( int i, const PCCPointSet3& cloud );
  void debugDiff( const PCCPointSet3& currPointCloud, const PCCPointSet3& refPointCloud, int i, int j );
  bool sameGeomPos( int i, int j, const PCCPointSet3& c1, const PCCPointSet3& c2 );

  void debugPoint( std::string fileName, const PCCPointSet3& p );

  bool matchingPointIsValid( const PCCPointSet3& currPointCloud, const PCCPointSet3& refPointCloud, int i, int j );

  uint32_t calculateColorDiff( const PCCPointSet3& currPointCloud, const PCCPointSet3& refPointCloud );
  bool     withinRange( const PCCPoint3D& p1, const PCCPoint3D& p2 );

  PCCPointSet3 reconstructPointCloud( const PCCPointSet3& currPointCloud, const PCCPointSet3& refPointCloud );
  PCCPointSet3 reconstructPointCloudFlipped( const PCCPointSet3& currPointCloud, const PCCPointSet3& refPointCloud );

  void writeToFile( std::string fileName, const PCCPointSet3& currPointCloud, const PCCPointSet3& refPointCloud );
  void writeFlippedToFile( std::string fileName, const PCCPointSet3& pi, const PCCPointSet3& pj );
  void writeAsPict(const PCCPointSet3& refPointCloud);

 private:
  int              threshold_;
  std::vector<int> matchingPointSet_;
  std::vector<int> matchingPointSetFlipped_;
  int              neiPoints_;
  int              dist_;
  PCCNNResult      refNei_, currNei_;
};

}  // namespace pcc

#endif
