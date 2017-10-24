#ifndef MEDUSA_H

#define MEDUSA_H

#include <fstream>

using namespace std;




const int frame_per_meta = 3; //meta per 100ms, 30fps video, -> 3 frame per meta

struct meta_struct{
	long timestamp;
	double light;
	double acc[3];//x,y,z
	double mag[3];//x,y,z
	double bearing[3];//orientation z x y, azimuth pitch roll
	double lat;
	double lon;
	double gps_acc;
};

//static struct meta_struct meta;
struct meta_struct read_metadata(struct meta_struct meta,ifstream& meta_file,int frame_id);
struct meta_struct pop_metadata(struct meta_struct meta,ifstream& meta_file);

#endif
