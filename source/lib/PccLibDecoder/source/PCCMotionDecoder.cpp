#include "PCCMotionDecoder.h"

using namespace pcc;

PCCPointSet3 PCCMotionDecoder::reconstructPointCloud( const PCCPointSet3& refPointCloud, int index) {
    std::cout << "Start\n";
    std::string f1 = "motion_stream";
    f1 += std::to_string(index);
    f1 += ".txt";

    std::cout << f1 << "\n"; 
    std::ifstream infile( f1 );
    
    PCCPointSet3 rec;

    int currPoint, refPoint, dx, dy, dz, dr, dg, db;
    while ( infile >> currPoint >> refPoint >> dx >> dy >> dz >> dr >> dg >> db) {
        //std::cout << currPoint << " " << refPoint << " " << dx << " " << dy << " " << dz << " " << dr << " " << dg << " " << db << "\n";
        PCCPoint3D p = refPointCloud[refPoint];
        PCCPoint3D nx = PCCPoint3D(p[0] + dx, p[1] + dy, p[2] + dz);

        PCCColor3B c = refPointCloud.getColor(refPoint); 
        PCCColor3B nxC = PCCColor3B(c[0] + dr, c[1] + dg, c[2] + db);
        rec.addPoint(nx, nxC);
    }  

    return rec;
}

//copies motion from a point cloud ahead of it for point cloud 1 -> 2
PCCPointSet3 PCCMotionDecoder::reconstructPointCloudSpecial( const PCCPointSet3& refPointCloud, int index) {
    std::cout << "Start Special\n";
    std::string f1 = "motion_stream";
    f1 += std::to_string(index);
    f1 += ".txt";

    vector<PCCColor3B> col;

    std::ifstream infile( f1 );
    int currPoint, refPoint, dx, dy, dz, dr, dg, db;
    while ( infile >> currPoint >> refPoint >> dx >> dy >> dz >> dr >> dg >> db) {
        //std::cout << currPoint << " " << refPoint << " " << dx << " " << dy << " " << dz << " " << dr << " " << dg << " " << db << "\n";
        PCCPoint3D p = refPointCloud[refPoint];
        PCCPoint3D nx = PCCPoint3D(p[0] + dx, p[1] + dy, p[2] + dz);

        PCCColor3B c = refPointCloud.getColor(refPoint); 
        PCCColor3B nxC = PCCColor3B(c[0] + dr, c[1] + dg, c[2] + db);

        col.push_back(nxC); 
    }  

    std::string f2 = "motion_stream";
    f2 += std::to_string(index + 1);
    f2 += ".txt";
    std::ifstream infile( f2 );
   
    while ( infile >> currPoint >> refPoint >> dx >> dy >> dz >> dr >> dg >> db) {


    }

}

