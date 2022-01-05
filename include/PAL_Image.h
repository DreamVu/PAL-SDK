# ifndef PAL_IMAGE_H
# define PAL_IMAGE_H


#include <sys/time.h>
#include <opencv2/core/mat.hpp>

//Wrapper structure to denote an image
namespace PAL
{
	struct Image
	{
		union RawData
		{
			void*           data;
			unsigned char*  u8_data;
			unsigned short* u16_data;
			float*          f32_data;
		}Raw;

		int rows;
		int cols;
		int channels;
		int bytesPerChannel;
		int stride;
		int size;
		timeval timestamp;
	
		Image() : rows(0), cols(0), channels(0), bytesPerChannel(0), stride(0), size(0) { Raw.data = 0; }

		Image(int width, int height) : rows(height), cols(width), channels(3),
			bytesPerChannel(1), stride(0), size(0) 
		{
			Raw.data = 0;
		}

		void SetDimensions(int width, int height, int channels, int bytesPerChannel);

		void Set(void* ptr, int width, int height, int channels = 3, int bytesPerChannel = 1);

		//Allocates new memory - depending on the dimensions and channels and bytesPerChannel.
		//Destroy should be called to cleanup this memory
		void Create(int width, int height, int channels = 3, int bytesPerChannel = 1);

		cv::Mat Convert();

		//This should be called only when when Create is called.
		void Destroy();
	};

}
# endif // PAL_IMAGE_H

