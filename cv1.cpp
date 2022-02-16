#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
//#include <cmath>

using namespace cv;
using namespace std;

//алгоритм сегментации
int segment(Mat image, int pixel_color)
{
	//гистограмма
	int histogram[256];	 
	int k = 0;
	cv::Vec3b value = 0;
	for (int v = 0; v < 256; v++)
	{
		k = 0;
		for (int i = 1; i < image.rows - 1; i++)
		{
			for (int j = 1; j < image.cols - 1; j++)
			{
				value = image.at<cv::Vec3b>(i, j);
				if (value[pixel_color] >= 255) { value[pixel_color] = 255; }
				if (value[pixel_color] <= 0) { value[pixel_color] = 255; }
				if (value[pixel_color] == v)
				{
					k++;
					histogram[v] = k;
				}
			}
		}
	}

	int TresholdValue[255];
	int temp = 0, temp2 = 0, temp3 = 0, temp4 = 0, temp5 = 0, temp6 = 0;
	int size = (image.rows * image.cols);

	int BWeight = 0, BMean = 0, BVariance = 0;
	int FWeight = 0, FMean = 0, FVariance = 0;

	int T[256];
	int ClassVariance = 0;

	for (int i = 0; i < 256; i++)
	{
		//относительная частота
		temp = temp + histogram[i];
		BWeight = temp / size;

		//среднийе ровни
		temp2 = temp2 + (i * histogram[i]);
		if (temp == 0) { temp = 1; }
		BMean = temp2 / temp;

		//максимальное значение оценки качества раздееления изображения
		temp3 = temp3 + (sqrt(i - BMean) * histogram[i]);
		BVariance = temp3 / temp;

		for (int j = i + 1; j < 256; j++)
		{
			temp4 = temp4 + histogram[j];
			FWeight = temp4 / size;

			temp5 = temp5 + (j * histogram[j]);
			if (temp4 == 0) temp4 = 1;
			FMean = temp5 / temp4;

			temp6 = temp6 + (sqrt(j - FMean) * histogram[j]);
			FVariance = temp6 / temp4;
		}
		//классовая дисперсия
		ClassVariance = (BWeight * BVariance + FWeight * FVariance);
		T[i] = ClassVariance;
	}

	//находим максимальное значение оценки качества разделения изображения на две части
	int MinNumber = T[1], Threshold = 0;

	for (int b = 1; b < 255; b++)
	{
		if (T[b] < MinNumber)
		{
			MinNumber = T[b];
			Threshold = b;//порог
		}
	}

	return Threshold;
}

//вывод результата
Mat printImage(Mat image, int pixel_color, int Threshold)
{
	for (int i = 0; i < image.rows; i++)
	{
		for (int j = 0; j < image.cols; j++)
		{
			cv::Vec3b color = image.at<cv::Vec3b>(i, j);

			if (color[pixel_color] > Threshold)
			{
				image.at<Vec3b>(Point(j, i)) = Vec3b(255, 0, 0);
			}
			else
			{
				image.at<Vec3b>(Point(j, i)) = Vec3b(0, 255, 0);
			}

		}
	}
	return image;
}

int main(int argc, char** argv)
{
	int pixel_color = 2; // 0 - b; 1- g; 2 - r;

	VideoCapture cap;
	cap.open("F:/Documents/test/cv1/VRS.avi");
	cv::Mat image, frame;

	// check if we succeeded
	if (!cap.isOpened()) {
		cerr << "ERROR! Unable to open camera\n";
		return -1;
	}

	while (1)
	{
		cap.read(image);
		if (image.empty())
			break;

		// вызова алгоритма сегментации
		int Threshold = segment(image, pixel_color);

		//рисуем итог
		frame = printImage(image, pixel_color, Threshold);

		imshow("Live", frame);

		if (waitKey(5) >= 0)
			break;
	}
}