#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <algorithm>
#include "render.h"
#include "constant.h"
#define width 400
#define height 300
using namespace std;
int main() {
  
  Render* tracer = new Render();
	tracer->Prepare_data(width, height);
  tracer->Start_rendering();
	char *file = "./output/1.ppm";
	
	std::ofstream ofs(file, std::ios::out | std::ios::binary);
	ofs << "P6\n" << width << " " << height << "\n255\n";
	for (unsigned i = 0; i < width * height; i++)
	{
		ofs << (unsigned char)(min(255,int(tracer->red_[i]))) <<
			(unsigned char)(min(255,int(tracer->green_[i]))) <<
			(unsigned char)(min(255,int(tracer->blue_[i])));
	}
	ofs.close();
}
