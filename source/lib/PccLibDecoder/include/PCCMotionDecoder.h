/*
 * This is the start of all code related to motion estimation
 * in the decoder WIP
 *
 */

#ifndef PCC_APP_MOTION_DECODER_H
#define PCC_APP_MOTION_DECODER_H

#include "PCCCommon.h"
#include "PCCPointSet.h"
#include "PCCImage.h"
#include "PCCKdTree.h"
#include <vector>
#include <map>

namespace pcc {
class PCCMotionDecoder {
 public:
  
  PCCMotionDecoder() {}; 
  ~PCCMotionDecoder() = default;

  PCCPointSet3 reconstructPointCloud( const PCCPointSet3& refPointCloud, int index );
  PCCPointSet3 reconstructPointCloudFlipped( const PCCPointSet3& refPointCloud, int index );
  PCCPointSet3 reconstructPointCloudColor( const PCCPointSet3& refPointCloud, int index );
  void debugPoint( std::string fileName, const PCCPointSet3& p ); 

 private:
};

}  // namespace pcc

#endif
