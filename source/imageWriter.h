#ifndef IMAGE_JFDJKDFSLJFDFKSDFDJFDFJSDKSFJSDLF
#define IMAGE_JFDJKDFSLJFDFKSDFDJFDFJSDKSFJSDLF
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/glut.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <lodepng.h>
#include "iostream"

//Image class
//This class can be used to write your final result to an image. 
//You can open the image using a PPM viewer.

//YOU CAN IGNORE THIS CODE!
 class RGBAValue
{
	public:
	RGBAValue(float rI=0, float gI=0, float bI=0, float aI=0)
	: r(rI)
	, g(gI)
	, b(bI)
	, a(aI)
	{
		if (r>1)
			r=1.0;
		if (g>1)
			g=1.0;
		if (b>1)
			b=1.0;
		if (a>1)
			a=1.0;

		if (r<0)
			r=0.0;
		if (g<0)
			g=0.0;
		if (b<0)
			b=0.0;
		if (a<0)
			a=0.0;
	};
	
	char operator[](int i) const
	{
		switch(i)
		{
			case 0:
				return (unsigned char)(r*255);
			case 1:
				return (unsigned char)(g*255);
			case 2:
				return (unsigned char)(b*255);
			case 3:
				return (unsigned char)(a*255);
			default: 
				return (unsigned char)(r*255);
		}
	}
	/*char & operator[](int i)
	{
		switch(i)
		{
			case 0:
				return r*255;
			case 1:
				return (unsigned char)g*255;
			case 2:
				return (unsigned char)b*255;
			case 3:
				return (unsigned char)a*255;
			default:
				return (unsigned char)r*255;
		}
	}*/
	float r, b,g,a;
};





class Image
{
	public:
	Image(int width, int height)
	: _width(width)
	, _height(height)
	{
		_image.resize(4*_width*_height);
	}
	void setPixel(int i, int j, const RGBAValue & rgba)
	{
		_image[4*(_width*j+i)]=rgba[0];
		_image[4*(_width*j+i)+1]=rgba[1];
		_image[4*(_width*j+i)+2]=rgba[2];
		_image[4*(_width*j+i)+3]=rgba[3];
		
	}
	std::vector<unsigned char> _image;
	int _width;
	int _height;

	bool writeImage(const char * filename);
};



bool Image::writeImage(const char * filename)
{
	//output .ppm file
	FILE* file;
    file = fopen(filename, "wb");
	if (!file)
	{
		printf("dump file problem... file\n");
		return false;
	}
	fprintf(file, "P6\n%i %i\n255\n",_width, _height);
	std::vector<unsigned char> imageC(3*_image.size()/4);
	unsigned int i=0;
	unsigned int j=0;
	while (i<_image.size()) {
		imageC[j]=_image[i];
		i++;
		j++;
		imageC[j]=_image[i];
		i++;
		j++;
		imageC[j]=_image[i];
		i++;
		j++;
		i++;
	}
	int t = fwrite(&(imageC[0]), _width * _height * 3, 1, file);
	if (t!=1)
	{
		printf("Dump file problem... fwrite\n");
		return false;
	}
	fclose(file);
	//output .png
	std::vector<unsigned char> png;
	unsigned error = lodepng::encode(png, _image, _width, _height);
	if(!error) {
		lodepng::save_file(png, "result.png");
		return true;
	}
	return false;
}

#endif
