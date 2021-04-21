#include "PCCMotionDecoder.h"

using namespace pcc;

PCCPointSet3 PCCMotionDecoder::reconstructPointCloud( const PCCPointSet3& refPointCloud, int index ) {
  std::cout << "Start\n";
  std::string f1 = "motion_stream";
  f1 += std::to_string( index );
  f1 += ".txt";

  std::cout << f1 << "\n";
  std::ifstream infile( f1 );

  PCCPointSet3 rec;
  std::map<int, std::vector<std::vector<int>>> colorInfo;
  int lossMotion = 1;

  if ( index == ( lossMotion + 1 ) || index == ( lossMotion + 2 ) ) {
    std::cout << "Start Propogation\n";
    std::string f1 = "motion_stream";
    f1 += std::to_string( 1 );
    f1 += ".txt";

    std::cout << f1 << "\n";
    std::ifstream infile( f1 );

    int currPoint, refPoint, dx, dy, dz, dr, dg, db;
    while ( infile >> currPoint >> refPoint >> dx >> dy >> dz >> dr >> dg >> db ) {
      PCCPoint3D p = refPointCloud[refPoint];

      int        mul = 1;
      PCCPoint3D nx  = PCCPoint3D( p[0] + mul * dx, p[1] + mul * dy, p[2] + mul * dz );

      PCCColor3B c   = refPointCloud.getColor( refPoint );
      PCCColor3B nxC = PCCColor3B( c[0] + dr, c[1] + dg, c[2] + db );
      colorInfo[currPoint].push_back( { refPoint, dr, dg, db } );
    }

    std::string f2 = "motion_stream";
    f2 += std::to_string( 2 );
    f2 += ".txt";

    std::cout << f2 << "\n";
    std::ifstream infile2( f2 );

    while ( infile2 >> currPoint >> refPoint >> dx >> dy >> dz >> dr >> dg >> db ) {
      if ( colorInfo.find( refPoint ) == colorInfo.end() ) {
        std::cout << "EMPTY\n";
        continue;
      }

      int diff = index - lossMotion + 1;

      for ( std::vector<int>& v : colorInfo[refPoint] ) {
        PCCPoint3D p  = refPointCloud[v[0]];
        PCCPoint3D nx = PCCPoint3D( p[0] + diff * dx, p[1] + diff * dy, p[2] + diff * dz );

        PCCColor3B c   = refPointCloud.getColor( v[0] );
        PCCColor3B nxC = PCCColor3B( c[0] + 0*v[1], c[1] + 0*v[2], c[2] + 0*v[3] );
        rec.addPoint( nx, nxC );
      }
    }

    return rec;
  }

  int currPoint, refPoint, dx, dy, dz, dr, dg, db;
  while ( infile >> currPoint >> refPoint >> dx >> dy >> dz >> dr >> dg >> db ) {
    // std::cout << currPoint << " " << refPoint << " " << dx << " " << dy << " " << dz << " " << dr << " " << dg << " "
    // << db << "\n";
    PCCPoint3D p  = refPointCloud[refPoint];

    int mul = 1;
    PCCPoint3D nx = PCCPoint3D( p[0] + mul*dx, p[1] + mul*dy, p[2] + mul*dz );

    PCCColor3B c   = refPointCloud.getColor( refPoint );
    PCCColor3B nxC = PCCColor3B( c[0] + dr, c[1] + dg, c[2] + db );

    if ( index != lossMotion ) { rec.addPoint( nx, nxC ); }

    colorInfo[currPoint].push_back( { refPoint, dr, dg, db } );
  }

  if ( index == lossMotion ) {
    std::cout << "Start Error Concealment\n";
    std::string f1 = "motion_stream";
    f1 += std::to_string( index + 1 );
    f1 += ".txt";

    std::cout << f1 << "\n";
    std::ifstream infile( f1 );

    while ( infile >> currPoint >> refPoint >> dx >> dy >> dz >> dr >> dg >> db ) {
      // std::cout << currPoint << " " << refPoint << " " << dx << " " << dy << " " << dz << " " << dr << " " << dg << "
      // " << db << "\n";
      if ( colorInfo.find( refPoint ) == colorInfo.end() ) {
        std::cout << "EMPTY\n";
        continue;
      }

      for ( std::vector<int>& v : colorInfo[refPoint] ) {
        PCCPoint3D p  = refPointCloud[v[0]];
        PCCPoint3D nx = PCCPoint3D( p[0] + dx, p[1] + dy, p[2] + dz );

        PCCColor3B c   = refPointCloud.getColor( v[0] );
        PCCColor3B nxC = PCCColor3B( c[0] + v[1], c[1] + v[2], c[2] + v[3] );
        rec.addPoint( nx, nxC );
      }
      //           std::vector<int> v = colorInfo[refPoint];
    }
  }

  return rec;
}

PCCPointSet3 PCCMotionDecoder::reconstructPointCloudColor( const PCCPointSet3& refPointCloud, int index ) {
  std::cout << "Start\n";
  std::string f1 = "motion_stream";
  f1 += std::to_string( index );
  f1 += ".txt";

  std::cout << f1 << "\n";
  std::ifstream infile( f1 );

  PCCPointSet3 rec;
  std::map<int, std::vector<std::vector<int>>> colorInfo;
  int lossTexture = 1;

  int currPoint, refPoint, dx, dy, dz, dr, dg, db;
  while ( infile >> currPoint >> refPoint >> dx >> dy >> dz >> dr >> dg >> db ) {
    // std::cout << currPoint << " " << refPoint << " " << dx << " " << dy << " " << dz << " " << dr << " " << dg << " "
    // << db << "\n";
    PCCPoint3D p  = refPointCloud[refPoint];

    int mul = 1;
    PCCPoint3D nx = PCCPoint3D( p[0] + mul*dx, p[1] + mul*dy, p[2] + mul*dz );

    PCCColor3B c   = refPointCloud.getColor( refPoint );
    
    int mul1 = 1;
    if ( index == lossTexture ) { mul1 = 0; }
    PCCColor3B nxC = PCCColor3B( c[0] + mul1 * dr, c[1] + mul1 * dg, c[2] + mul1*db );
    rec.addPoint( nx, nxC );
    colorInfo[currPoint].push_back( { refPoint, dx, dy, dz } );
  }

  if ( index == 1 ) {
    std::cout << "Start Error Concealment\n";
    std::string f1 = "motion_stream";
    f1 += std::to_string( index + 1 );
    f1 += ".txt";

    std::cout << f1 << "\n";
    std::ifstream infile( f1 );

    while ( infile >> currPoint >> refPoint >> dx >> dy >> dz >> dr >> dg >> db ) {
      // std::cout << currPoint << " " << refPoint << " " << dx << " " << dy << " " << dz << " " << dr << " " << dg << "
      // " << db << "\n";
      if ( colorInfo.find( refPoint ) == colorInfo.end() ) {
        std::cout << "EMPTY\n";
        continue;
      }

      for ( std::vector<int>& v : colorInfo[refPoint] ) {
        PCCPoint3D p  = refPointCloud[v[0]];
        PCCPoint3D nx = PCCPoint3D( p[0] + v[1], p[1] + v[2], p[2] + v[3] );

        PCCColor3B c   = refPointCloud.getColor( v[0] );
        PCCColor3B nxC = PCCColor3B( c[0] + dr, c[1] + dg, c[2] + db );
        rec.addPoint( nx, nxC );
      }
      //           std::vector<int> v = colorInfo[refPoint];
    }
  }

  return rec;
}

PCCPointSet3 PCCMotionDecoder::reconstructPointCloudFlipped( const PCCPointSet3& refPointCloud, int index ) {
  std::cout << "Start\n";
  std::string f1 = "flipped_motion_stream";
  f1 += std::to_string( index );
  f1 += ".txt";

  std::cout << f1 << "\n";
  std::ifstream infile( f1 );

  PCCPointSet3 rec;

  std::map<int, std::vector<std::vector<int>>> colorInfo;

  int lossMotion = -1;

  int currPoint, refPoint, dx, dy, dz, dr, dg, db;
  while ( infile >> refPoint >> currPoint >> dx >> dy >> dz >> dr >> dg >> db ) {
    // std::cout << currPoint << " " << refPoint << " " << dx << " " << dy << " " << dz << " " << dr << " " << dg << " "
    // << db << "\n";
    PCCPoint3D p  = refPointCloud[refPoint];

    int mul = 1;
    PCCPoint3D nx = PCCPoint3D( p[0] + mul*dx, p[1] + mul*dy, p[2] + mul*dz );

    PCCColor3B c   = refPointCloud.getColor( refPoint );
    PCCColor3B nxC = PCCColor3B( c[0] + dr, c[1] + dg, c[2] + db );

    if ( index != lossMotion ) { rec.addPoint( nx, nxC ); }

    colorInfo[currPoint].push_back( { refPoint, dr, dg, db } );
  }

  if ( index == lossMotion ) {
    std::cout << "Start Error Concealment\n";
    std::string f1 = "flipped_motion_stream";
    f1 += std::to_string( index + 1 );
    f1 += ".txt";

    std::cout << f1 << "\n";
    std::ifstream infile( f1 );

    while ( infile >> refPoint >> currPoint >> dx >> dy >> dz >> dr >> dg >> db ) {
      // std::cout << currPoint << " " << refPoint << " " << dx << " " << dy << " " << dz << " " << dr << " " << dg << "
      // " << db << "\n";
      if ( colorInfo.find( refPoint ) == colorInfo.end() ) {
        std::cout << "EMPTY\n";
        continue;
      }

      for ( std::vector<int>& v : colorInfo[refPoint] ) {
        PCCPoint3D p  = refPointCloud[v[0]];
        PCCPoint3D nx = PCCPoint3D( p[0] + dx, p[1] + dy, p[2] + dz );

        PCCColor3B c   = refPointCloud.getColor( v[0] );
        PCCColor3B nxC = PCCColor3B( c[0] + v[1], c[1] + v[2], c[2] + v[3] );
        rec.addPoint( nx, nxC );
      }
      //           std::vector<int> v = colorInfo[refPoint];
    }
  }

  return rec;
}

void PCCMotionDecoder::debugPoint( std::string fileName, const PCCPointSet3& p ) {
  std::ofstream outfile( fileName );
  int           N = p.getPointCount();

  for ( int i = 0; i < N; i++ ) {
    outfile << p[i][0] << " " << p[i][1] << " " << p[i][2] << " ";

    auto c = p.getColor( i );
    outfile << (int)c[0] << " " << (int)c[1] << " " << (int)c[2] << "\n";
  }

  outfile.close();
}
