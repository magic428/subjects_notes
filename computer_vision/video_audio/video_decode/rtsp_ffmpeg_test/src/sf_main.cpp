/**
 * The main function is an example of video dehazing
 * 
 * Last updated: 2019-10-09
 * Author: Zhijie Guo. 
 */


#include "stream_fetch.h"

using namespace tdmc;

int main(int argc, char** argv)
{
	// cv::VideoCapture cap = cv::VideoCapture("rtsp://admin:admin123@10.224.41.61:554/h264/ch1/main/av_stream");
	std::string endpoint = "rtsp://admin:root1234@192.168.1.64:554/h264/ch1/main/av_stream";
	//std::string endpoint = "rtsp://admin:root1234@192.168.1.64:554";
	//std::string endpoint = "rtsp://admin:admin123@192.168.1.164:554/h264/ch1/main/av_stream";

	cv::VideoCapture cap = cv::VideoCapture(endpoint);
	if (!cap.isOpened()) {
		cout << "Rtsp video stream open failed, check it: " << endpoint << endl;
	}

#if 0
	cv::Mat convas, show;
	while (1) {
		cap >> convas;
		cv::resize(convas, show, cv::Size(1180, 680));
		cv::imshow("demo", show);
		cv::waitKey(200);
	}
#endif

#if 1
	cout << "Rtsp video stream: " << endpoint << endl;
	Stream_Fetch sf(endpoint);


	cv::Mat convas, show;
	int key = 0;
	int i = 0;
	std::string pic_name = "demo_";
	std::string post_fix = ".jpg";
	//while (i++ < 2000) {
	while (1) {
		sf.get_one_frame(convas);

		if (!convas.data) {
			key = cv::waitKey(200);
			cout << "[main] no data: " << sf.frame_size() << endl;
		}
		else {
			cv::resize(convas, show, cv::Size(1180,680));
			cv::imshow("demo", show);
			//cv::imwrite(pic_name + std::to_string(i) + post_fix, show);
			key = cv::waitKey(10);

		}

		if (27 == key)
			break;
	}

#endif
	cv::destroyAllWindows();

	return 0;
}

