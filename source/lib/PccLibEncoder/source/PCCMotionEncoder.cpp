#include "PCCMotionEncoder.h"

using namespace pcc;

bool PCCMotionEncoder::buildMatchingPoints( const PCCPointSet3& currPointCloud, const PCCPointSet3& refPointCloud ) {
  int refCount  = refPointCloud.getPointCount();
  int currCount = currPointCloud.getPointCount();

  PCCKdTree currTree( currPointCloud );
  PCCKdTree refTree( refPointCloud );

  matchingPointSet_.resize( refCount );

  for ( int i = 0; i < refCount; i++ ) {
    const PCCPoint3D& refPoint = refPointCloud[i];
    refNei_                    = PCCNNResult();
    refTree.search( refPoint, neiPoints_, refNei_ );

    uint32_t bestDiff      = INT32_MAX - 1;
    int      matchingPoint = -1;

    for ( int j = 0; j < currCount; j++ ) {
      const PCCPoint3D& currPoint = currPointCloud[j];
      if ( !withinRange( refPoint, currPoint ) ) { continue; }

      currNei_ = PCCNNResult();
      currTree.search( currPoint, neiPoints_, currNei_ );
      uint32_t diff = calculateColorDiff( currPointCloud, refPointCloud );

      if ( diff < bestDiff ) {
        bestDiff      = diff;
        matchingPoint = j;
      }
    }

    if ( matchingPoint != -1 && !matchingPointIsValid( currPointCloud, refPointCloud, i, matchingPoint ) ) { matchingPoint = -1; }

    //debugDiff( currPointCloud, refPointCloud, i, matchingPoint );
    matchingPointSet_[i] = matchingPoint;
  
    if (i && i % 100 == 0) {
      std::cout << "encoding point " << i << "....\n";
    }
  }

  int totalMatched = refCount - std::count( matchingPointSet_.begin(), matchingPointSet_.end(), -1 );
  return totalMatched >= threshold_;
}

bool PCCMotionEncoder::matchingPointIsValid( const PCCPointSet3& currPointCloud,
                                             const PCCPointSet3& refPointCloud,
                                             int                 i,
                                             int                 j ) {
  auto     c1 = refPointCloud.getColor( i );
  auto     c2 = currPointCloud.getColor( j );
  uint16_t dr = std::max( c1[0], c2[0] ) - std::min( c1[0], c2[0] );
  uint16_t dg = std::max( c1[1], c2[1] ) - std::min( c1[1], c2[1] );
  uint16_t db = std::max( c1[2], c2[2] ) - std::min( c1[2], c2[2] );

  const uint16_t mx = ( 1 << 8 );
  return dr < mx && dg < mx && db < mx;
}

void PCCMotionEncoder::debugDiff( const PCCPointSet3& currPointCloud,
                                  const PCCPointSet3& refPointCloud,
                                  int                 i,
                                  int                 j ) {
  std::cout << "[" << i << "," << j << "]";
  if ( j == -1 ) return;

  auto d1 = refPointCloud[i];
  auto d2 = currPointCloud[j];
  auto mv = d1 - d2;

  std::cout << " MV: ";

  std::cout << "(" << (int)d1[0] << ", " << (int)d1[1] << ", " << (int)d1[2] << ")";

  std::cout << " - ";

  std::cout << "(" << (int)d2[0] << ", " << (int)d2[1] << ", " << (int)d2[2] << ")";

  std::cout << " = ";

  std::cout << "(" << (int)mv[0] << ", " << (int)mv[1] << ", " << (int)mv[2] << ")";

  auto     c1 = refPointCloud.getColor( i );
  auto     c2 = currPointCloud.getColor( j );
  uint16_t dr = std::max( c1[0], c2[0] ) - std::min( c1[0], c2[0] );
  uint16_t dg = std::max( c1[1], c2[1] ) - std::min( c1[1], c2[1] );
  uint16_t db = std::max( c1[2], c2[2] ) - std::min( c1[2], c2[2] );

  std::cout << " DT: ";
  std::cout << "(" << (int)c1[0] << ", " << (int)c1[1] << ", " << (int)c1[2] << ")";
  std::cout << " - ";
  std::cout << "(" << (int)c2[0] << ", " << (int)c2[1] << ", " << (int)c2[2] << ")";
  std::cout << " = ";
  std::cout << "(" << (int)dr << ", " << (int)dg << ", " << (int)db << ")";
  std::cout << "\n";
}

void PCCMotionEncoder::debugPos( int i, const PCCPointSet3& cloud ) {
  auto px = cloud[i];
  std::cout << "(" << px[0] << "," << px[1] << "," << px[2] << ")";

  auto cx = cloud.getColor( i );
  std::cout << "(" << (int)cx[0] << "," << (int)cx[1] << "," << (int)cx[2] << ")";
}

bool PCCMotionEncoder::sameGeomPos( int i, int j, const PCCPointSet3& c1, const PCCPointSet3& c2 ) {
  auto px1 = c1[i];
  auto px2 = c2[j];
  return px1[0] == px2[0] && px1[1] == px2[1] && px1[2] == px2[2];
}

uint32_t PCCMotionEncoder::calculateColorDiff( const PCCPointSet3& currPointCloud, const PCCPointSet3& refPointCloud ) {
  uint32_t sum = 0;

  for ( int i = 0; i < neiPoints_; i++ ) {
    int p1 = currNei_.indices( i );
    int p2 = refNei_.indices( i );

    const PCCColor3B& c1 = currPointCloud.getColor( p1 );
    const PCCColor3B& c2 = refPointCloud.getColor( p2 );

    uint16_t dr = std::max( c1[0], c2[0] ) - std::min( c1[0], c2[0] );
    uint16_t dg = std::max( c1[1], c2[1] ) - std::min( c1[1], c2[1] );
    uint16_t db = std::max( c1[2], c2[2] ) - std::min( c1[2], c2[2] );

    uint32_t sx = dr + dg + db;
    sum += sx * sx;
  }

  return sum;
}

bool PCCMotionEncoder::withinRange( const PCCPoint3D& p1, const PCCPoint3D& p2 ) {
  long long dx = p1[0] - p2[0];
  long long dy = p1[1] - p2[1];
  long long dz = p1[2] - p2[2];
  return ( dx * dx + dy * dy + dz * dz ) <= dist_ * dist_;
}

void PCCMotionEncoder::writeToFile( std::string fileName ) {
  std::ofstream outfile( fileName );
  for ( int i = 0; i < (int) matchingPointSet_.size(); i++ ) { 
    outfile << i << " " << matchingPointSet_[i] << std::endl; 
  }
  outfile.close();
}

void PCCMotionEncoder::writeAsPict(const PCCPointSet3& refPointCloud) {
  //HACK: read from file here to check
  //why? cos we cant wait 10 years for the script to encode again
  std::ifstream infile( "act_motion_stream1.txt" );
  
  int refCount  = refPointCloud.getPointCount();
  matchingPointSet_.resize( refCount );

  int a, b;
  while ( infile >> a >> b) {
    matchingPointSet_[a] = b;
  }  

  //how to find Pic location??
  PCCImage<uint16_t, 3> test;

  for ( int i = 0; i < (int) matchingPointSet_.size(); i++ ) {
    auto p = refPointCloud.getPointPatchIndex(i);

    //use p to write as picture
    std::cout << i << " " << p.first << " " << p.second << "\n";
  } 

}

 PCCPointSet3 PCCMotionEncoder::reconstructPointCloud( const PCCPointSet3& currPointCloud, const PCCPointSet3& refPointCloud ) {
    PCCPointSet3 rec;
    for (int i = 0; i < (int) matchingPointSet_.size(); i++) {
      if (matchingPointSet_[i] == -1) continue;
      int m = matchingPointSet_[i];
      rec.addPoint(currPointCloud[m], currPointCloud.getColor(m));  
    }
    return rec;
}


