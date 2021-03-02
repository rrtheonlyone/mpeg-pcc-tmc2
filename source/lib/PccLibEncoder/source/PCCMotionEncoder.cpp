#include "PCCMotionEncoder.h"

using namespace pcc;

bool PCCMotionEncoder::genMotionData( const PCCPointSet3& currPointCloud, const PCCPointSet3& refPointCloud ) {
  int refCount  = refPointCloud.getPointCount();
  int currCount = currPointCloud.getPointCount();

  PCCKdTree currTree( currPointCloud );
  PCCKdTree refTree( refPointCloud );

  matchingPointSet_.resize( refCount );

  refNei_  = PCCNNResult();
  currNei_ = PCCNNResult();

  for ( int i = 0; i < refCount; i++ ) {
    const PCCPoint3D& refPoint = refPointCloud[i];
    refTree.search( refPoint, neiPoints_, refNei_ );

    uint32_t bestDiff      = 1000;
    int      matchingPoint = -1;

    for ( int j = 0; j < currCount; j++ ) {
      const PCCPoint3D& currPoint = currPointCloud[i];
      if ( !withinRange( refPoint, currPoint ) ) { continue; }

      currTree.search( currPoint, neiPoints_, currNei_ );
      uint16_t diff = calculateColorDiff( currPointCloud, refPointCloud );

      if ( diff < bestDiff ) {
        bestDiff      = diff;
        matchingPoint = j;
      }
    }

    std::cout << "Match " << i << " with " << matchingPoint << ": [" << (int)bestDiff << "]\n";

    if ( matchingPoint != -1 ) {
      std::cout << ( refPoint - currPointCloud[matchingPoint] );

      auto c1 = refPointCloud.getColor(i);
      auto c2 = currPointCloud.getColor(matchingPoint);
      std::cout << (int)c1[0] << " " << (int)c1[1] << " " << (int)c1[2] << std::endl;
      std::cout << (int)c2[0] << " " << (int)c2[1] << " " << (int)c2[2] << std::endl;

      auto dt = ( refPointCloud.getColor( i ) - currPointCloud.getColor( matchingPoint ) );
      std::cout << (int)dt[0] << " " << (int)dt[1] << " " << (int)dt[2] << std::endl;
    }

    matchingPointSet_[i] = matchingPoint;
  }

  int totalMatched = refCount - std::count( matchingPointSet_.begin(), matchingPointSet_.end(), 0 );
  return totalMatched >= threshold_;
}

uint32_t PCCMotionEncoder::calculateColorDiff( const PCCPointSet3& currPointCloud, const PCCPointSet3& refPointCloud ) {
  uint32_t sum = 0;
  for ( int i = 0; i < neiPoints_; i++ ) {
    const PCCColor3B& c1 = currPointCloud.getColor( currNei_.indices( i ) );
    const PCCColor3B& c2 = refPointCloud.getColor( refNei_.indices( i ) );
    sum += ( c1 - c2 ).getNorm2();
  }

  return sum;
}

bool PCCMotionEncoder::withinRange( const PCCPoint3D& p1, const PCCPoint3D& p2 ) {
  return ( p1 - p2 ).getNorm2() <= dist_ * dist_;
}
