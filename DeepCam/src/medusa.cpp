#include "DeepCam/include/medusa.h"
#include <iostream>
#include <fstream>

using namespace std;

static int meta_id = 0;

// read the file stream with accodring to frame id, update only per 3 frames for 30fps vidoe
struct meta_struct read_metadata(struct meta_struct meta,ifstream& meta_file, int frame_id){

	if (meta_file.eof()) {
		cerr<<"metadata file end" << endl;
		return meta;
	}
	//update only per 3 frame
	if (frame_id % frame_per_meta == 0){
		meta_file >> meta.timestamp;
		// cout << "meta timstamp" << meta.timestamp << endl;
		meta_file >> meta.light;
		int i=0;
		for (i=0;i<3;i++){
			meta_file >> meta.acc[i];
		}
		for (i=0;i<3;i++){
			meta_file >> meta.mag[i];
		}
		for (i=0;i<3;i++){
			meta_file >> meta.bearing[i];
		}
		meta_file >> meta.lon;
		meta_file >> meta.lat;
		meta_file >> meta.gps_acc;
		meta_id++;

	}


	// cout << "Metadata # "<< meta_id;

	return meta;
}


// read to clear metadata file in front without doing nothing
struct meta_struct pop_metadata(struct meta_struct meta, ifstream& meta_file){
	if (meta_file.eof()) {
		cerr<<"metadata file end" << endl;
		return meta;
	}
	//update only per 3 frame
		meta_file >> meta.timestamp;
		meta_file >> meta.light;
		int i=0;
		for (i=0;i<3;i++){
			meta_file >> meta.acc[i];
		}
		for (i=0;i<3;i++){
			meta_file >> meta.mag[i];
		}
		for (i=0;i<3;i++){
			meta_file >> meta.bearing[i];
		}
		meta_file >> meta.lon;
		meta_file >> meta.lat;
		meta_file >> meta.gps_acc;
		meta_id++;


//	cout << "Metadata # "<< meta_id;

	return meta;
}
