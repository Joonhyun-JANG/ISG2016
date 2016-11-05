#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <iomanip>

#include <wiringPi.h>
#include <wiringSerial.h>

#include <stdio.h>
#include <string.h>
#include <math.h>


using namespace std;
using namespace cv;

#define WIDTH	320
#define HEIGHT	240
#define PI 3.1415926


Mat src, src_gray, src_gray_tmp, img_scene, src_HSV, src_H, src_S, src_V, src_ball, src_ball_gray;
Mat milk1;
Mat dst, detected_edges, ROI;

int fd;
int edgeThresh = 1, G_x_min = 320, G_x_max = 0, G_y_min = 240, G_y_max = 0;
int lowThreshold;
int BlackThres = 0, WhiteThres = 255, lowRedThres = 0, highRedThres = 0, lowGreenThres = 0, highGreenThres = 0;
int diffRedThres = 0, diffGreenThres = 0;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
int green_cnt[3][4] = { 0, }, red_cnt[6][8] = { 0, }, edge_distance[7] = { 10,60,110,160,210,260,310 }, edge_L = 0, edge_R = 0;
int ball_cnt_x[HEIGHT]={0,}, ball_cnt_y[WIDTH]={0,};
int a = 0, look_at = 0;
VideoCapture cap(-1);


int cannyThreshold = 200, maxCannyThreshold = 255;
int accumulatorThreshold = 50, maxAccumulatorThreshold = 200;

    const std::string windowName = "Hough Circle Detection Demo";
    const std::string cannyThresholdTrackbarName = "Canny threshold";
    const std::string accumulatorThresholdTrackbarName = "Accumulator Threshold";


FILE *fp;

void load_settings();
void save_settings();
void detect_ball();
void HoughDetection(const Mat& src_gray, const Mat& src_display, int cannyThreshold, int accumulatorThreshold);


/**
* @function CannyThreshold
* @brief Trackbar callback - Canny thresholds input with a ratio 1:3
*/


// 화면에 선 그려주는 함수
void MyLine(Mat img, Point start, Point end)
{
	int thickness = 2;
	int lineType = 8;
	line(img,
		start,
		end,
		Scalar(0, 0, 255),
		thickness,
		lineType);
}

void on_trackbar(int, void*)
{

}

// 외곽선 검출을 위한 Canny Edge detector
void CannyThreshold(int, void*)
{
	/// Reduce noise with a kernel 3x3
	blur(src_gray, detected_edges, Size(3, 3));

	/// Canny detector
	Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);

	/// Using Canny's output as a mask, we display our result
	dst = Scalar::all(0);

	src.copyTo(dst, detected_edges);
}



int initialize() {
	if ((fd = serialOpen("/dev/serial0", 4800)) < 0)
	{
		printf("Unable to open serial device");
		return -1;
	}

	if (wiringPiSetup() == -1)
	{
		printf("Unable to load wiringPi");
		return -1;
	}

	if (!cap.isOpened())
	{
		cout << "Cannot open camera" << endl;
		return -1;
	}
	cap.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);

	return 1;
}

void make_windows(){
	namedWindow("find_ball", WINDOW_AUTOSIZE );
	createTrackbar(cannyThresholdTrackbarName, "find_ball", &cannyThreshold,maxCannyThreshold);
    createTrackbar(accumulatorThresholdTrackbarName, "find_ball", &accumulatorThreshold, maxAccumulatorThreshold);

}
 
int main(int argc, char** argv) {
	if (initialize() == -1) return -1;
	load_settings();
	make_windows();

	// Create a window
	namedWindow("Edge Map", CV_WINDOW_AUTOSIZE);
	namedWindow("Output", CV_WINDOW_AUTOSIZE);

	serialPutchar(fd, 62); // init for looking foward


	while (1)
	{
		bool bSuccess = cap.read(src);

		if (!bSuccess)
		{
			cout << "Cannot read a frame from camera" << endl;
			break;
		}

		cvtColor(src, src_gray, CV_BGR2GRAY);
		cvtColor(src, src_HSV, COLOR_BGR2HSV);
		src.copyTo(src_ball);

		// Create a matrix of the same type and size as src (for dst)
		dst.create(src.size(), src.type());
		src_gray_tmp.create(src_gray.size(), src_gray.type());
		createTrackbar("Min Threshold:", "Edge Map", &lowThreshold, max_lowThreshold, CannyThreshold);

		// Create a Trackbar for user to enter threshold
		//  createTrackbar( "Max Threshold:", "Edge Map", &lowThreshold, max_lowThreshold, CannyThreshold );
		// Show the image
		CannyThreshold(0, 0);

		int cnt[7] = { 0, };
		for (int i = 0; i<6; i++) {
			for (int j = 0; j<8; j++) {
				green_cnt[i][j] = 0;
				red_cnt[i][j] = 0;
			}
		}

		for (int i = 0; i<7; i++) {
			for (int j = 200 - 1; j >= 0; j--) {
				if ((int)dst.at<Vec3b>(j, edge_distance[i]).val[0]<30 && (int)dst.at<Vec3b>(j, edge_distance[i]).val[1]<30 && (int)dst.at<Vec3b>(7, edge_distance[i]).val[2]<30) cnt[i]++;
				else break;
			}
		}

		for (int i = 0; i<7; i++) {
			MyLine(dst, Point(edge_distance[i], 199 - cnt[i]), Point(edge_distance[i], 199));
		}
		edge_L = (cnt[0] + cnt[1] + cnt[2]) / 3;
		edge_R = (cnt[3] + cnt[4] + cnt[5]) / 3;

		stringstream ss_L;
		ss_L << edge_L;
		string str_L = ss_L.str();
		putText(dst, str_L, Point2f(130, 100), FONT_HERSHEY_PLAIN, 0.7, Scalar(255, 255, 255), 1, 8, false);

		stringstream ss_R;
		ss_R << edge_R;
		string str_R = ss_R.str();
		putText(dst, str_R, Point2f(185, 100), FONT_HERSHEY_PLAIN, 0.7, Scalar(255, 255, 255), 1, 8, false);

		imshow("Edge Map", dst);
		imshow("Output", src);


		on_trackbar(0, 0);
		int tmpt;
		tmpt = BlackThres;
		G_x_min = WIDTH, G_x_max = 0, G_y_min = HEIGHT, G_y_max = 0;

		for (int i = 0; i<HEIGHT; i++) {
			for (int j = 0; j<WIDTH; j++) {
				if (src_HSV.at<Vec3b>(i, j).val[0]<lowRedThres || src_HSV.at<Vec3b>(i, j).val[0]>highRedThres) {
				// find RED
					int BG_tmp;
					BG_tmp = (src.at<Vec3b>(i, j).val[0] + src.at<Vec3b>(i, j).val[1]) / 2;
					if ((src.at<Vec3b>(i, j).val[2] - BG_tmp)>diffRedThres) {
						src.at<Vec3b>(i, j).val[0] = 0;
						src.at<Vec3b>(i, j).val[1] = 0;
						src.at<Vec3b>(i, j).val[2] = 255;
						red_cnt[i / 40][j / 40]++;

						//ball_cnt_x[i]++;
						//ball_cnt_y[j]++;
						src_ball.at<Vec3b>(i, j).val[0] = 255;
						src_ball.at<Vec3b>(i, j).val[1] = 255;
						src_ball.at<Vec3b>(i, j).val[2] = 255;
					}
					else {
						src.at<Vec3b>(i, j).val[0] = 0;
						src.at<Vec3b>(i, j).val[1] = 0;
						src.at<Vec3b>(i, j).val[2] = 0;

					}
				}
				else if (lowGreenThres < src_HSV.at<Vec3b>(i, j).val[0] && src_HSV.at<Vec3b>(i, j).val[0]<highGreenThres) {
					int RB_tmp;
					RB_tmp = (src.at<Vec3b>(i, j).val[0] + src.at<Vec3b>(i, j).val[2]) / 2;
					if ((src.at<Vec3b>(i, j).val[1] - RB_tmp)>diffGreenThres) {
						src.at<Vec3b>(i, j).val[0] = 0;
						src.at<Vec3b>(i, j).val[1] = 255;
						src.at<Vec3b>(i, j).val[2] = 0;
						green_cnt[i / 80][j / 80]++;
						if (G_x_min>j) G_x_min = j;
						if (G_x_max<j) G_x_max = j;
						if (G_y_min>i) G_y_min = i;
						if (G_y_max<i) G_y_max = i;
					}
					else {
						src.at<Vec3b>(i, j).val[0] = 0;
						src.at<Vec3b>(i, j).val[1] = 0;
						src.at<Vec3b>(i, j).val[2] = 0;
					}

				}
				else {
					src.at<Vec3b>(i, j).val[0] = 0;
					src.at<Vec3b>(i, j).val[1] = 0;
					src.at<Vec3b>(i, j).val[2] = 0;
				}

				if (src_gray.at<unsigned char>(i, j)<BlackThres) {
					src_gray.at<unsigned char>(i, j) = 0;
				}
				else if (src_gray.at<unsigned char>(i, j)>WhiteThres) {
					src_gray.at<unsigned char>(i, j) = 255;
				}
				else {
					src_gray.at<unsigned char>(i, j) = 128;
				}
			}
		}

		detect_ball();

		//ROI = fullImage(Rect(G_x_min, G_y_min, G_x_max-G_x_min, G_y_max-G_y_min));
		if (G_x_max - G_x_min>0 && G_y_max - G_y_min>0) {
			src_gray(Rect(G_x_min, G_y_min, G_x_max - G_x_min, G_y_max - G_y_min)).copyTo(ROI);
			imshow("ROI", ROI);
			//cout << "a";
		}
		for (int i = 80; i<WIDTH; i += 80) {
			line(src, Point2f(i, 0), Point2f(i, HEIGHT-1), Scalar(255, 0, 0), 1);
		}
		for (int j = 80; j<HEIGHT; j += 80) {
			line(src, Point2f(0, j), Point2f(WIDTH-1, j), Scalar(255, 0, 0), 1);
		}

		int green_max = 0, green_max_index[2] = { 0, };
		int red_max = 0, red_max_index[2] = { 0, };
		for (int i = 0; i<3; i++) {
			for (int j = 0; j<4; j++) {
				if (green_max < green_cnt[i][j]) {
					green_max_index[0] = i;
					green_max_index[1] = j;
					green_max = green_cnt[i][j];
				}
				if (i<6 - 1 && j<8 - 1 && red_max<(red_cnt[i][j] + red_cnt[i][j + 1] + red_cnt[i + 1][j] + red_cnt[i + 1][j + 1])) {
					red_max_index[0] = i;
					red_max_index[1] = j;
					red_max = red_cnt[i][j] + red_cnt[i][j + 1] + red_cnt[i + 1][j] + red_cnt[i + 1][j + 1];
				}
				stringstream ss;
				ss << green_cnt[i][j];
				string str = ss.str();
				putText(src, str, Point2f(j * 80, (i + 1) * 80), FONT_HERSHEY_PLAIN, 0.7, Scalar(255, 255, 255), 1, 8, false);
			}
		}


		rectangle(src, Point(green_max_index[1] * 80, green_max_index[0] * 80), Point(green_max_index[1] * 80 + 80, green_max_index[0] * 80 + 80), Scalar(128, 128, 0), 0, 8);
	
		if (red_max>1700) {
			rectangle(src, Point(red_max_index[1] * 40, red_max_index[0] * 40), Point(red_max_index[1] * 40 + 80, red_max_index[0] * 40 + 80), Scalar(0, 255, 255), 0, 8);
			//cout << red_max_index[0] << " " << red_max_index[1] << endl;
			if (red_max_index[1] == 3 && a == 0) {
				serialPutchar(fd, 84);
				a = -1;
				//cout << "go to the ball" << endl;
			}
		}
		//cout << "!" << a << endl;
		//cout << "i: " << green_max_index[0] << " j: " << green_max_index[1] << " max: " << green_max << endl;
		if (green_max_index[1]<3 && green_max>100 && a == 0) {
			serialPutchar(fd, 49); // 
			a = -1;
			//cout << "move left" << endl;
		}
		if (green_max_index[0]>3 && green_max>100 && a == 0 && look_at == 0) {
			serialPutchar(fd, 62); // 
			a = -1;
			look_at = -1;
			//cout << "look at down" << endl;
		}

		if (green_max_index[1]>4 && green_max>100 && a == 0) {
			serialPutchar(fd, 42); // 
			a = -1;
			//cout << "move right" << endl;
		}

		if (look_at != 0 && green_max < 100 && a == 0) {
			serialPutchar(fd, 61); // 
			a = -1;
			look_at = 0;
			//cout << "look at foward" << endl;
		}
		if (look_at == -1 && green_max > 100 && a == 0 && (green_max_index[1] == 3 || green_max_index[1] == 4) && green_max_index[0]<2) {
			serialPutchar(fd, 65);
			a = -1;
			//cout << "move foward" << endl;
		}
		if (look_at == 0 && green_max > 100 && a == 0 && green_max_index[0] <= 3) {
			serialPutchar(fd, 65);
			a = -1;
			//cout << "move foward" << endl;
		}
		if (look_at == -1 && green_max > 100 && a == 0 && (green_max_index[1] == 3 || green_max_index[1] == 4) && (green_max_index[0] >= 2)) {
			serialPutchar(fd, 71); // 
			a = -1;
			//cout << "hold milk" << endl;
		}
		if (edge_L - edge_R>20) {
			serialPutchar(fd, 37); // 
			a = -1;
			//cout << "Right" << endl;
		}
		if (edge_R - edge_L>20) {
			serialPutchar(fd, 44); // 
			a = -1;
			//cout << "Left" << endl;

		}
		imshow("Hue", src);
		
		createTrackbar("Red Min:", "Hue", &lowRedThres, 255, on_trackbar);
		createTrackbar("Red Max:", "Hue", &highRedThres, 255, on_trackbar);
		createTrackbar("Red Diff:", "Hue", &diffRedThres, 255, on_trackbar);
		createTrackbar("Green Min:", "Hue", &lowGreenThres, 255, on_trackbar);
		createTrackbar("Green Max:", "Hue", &highGreenThres, 255, on_trackbar);
		createTrackbar("Green Diff:", "Hue", &diffGreenThres, 255, on_trackbar);

		threshold(src, src, 0, BlackThres, CV_THRESH_BINARY);
		imshow("Grayscale", src_gray);
		createTrackbar("Black Threshold:", "Grayscale", &BlackThres, 255, on_trackbar);
		createTrackbar("White Threshold:", "Grayscale", &WhiteThres, 255, on_trackbar);

		//serialPutchar (fd, (unsigned char)i);



		while (serialDataAvail(fd) && a != 0)
		{
			//printf("ready\n");
			printf("!%d\n", fd);
			a = serialGetchar(fd);
			//printf(" -> %3d\n", a);
			a = -2;
			fflush(stdout);
		}
		if (a == -2) {
			a = 0;
		}

		//	fflush (stdout) ;
		if (waitKey(30) == 27)
		{
			printf("key\n");
			cout << "Exit" << endl;
			//serialPutchar (fd, (unsigned char)7);
			break;
		}

	}
	save_settings();

	return 0;
}


void detect_ball(){
/*	int i_max=0, i_max_i=0, j_max=0, j_max_i=0;
	for(int j=0;j<WIDTH;j++){
		if(j_max<=ball_cnt_y[j]){
				j_max = ball_cnt_y[j];
				j_max_i = j;		
		}
		ball_cnt_y[j] = 0;
	}
*/



	
	for(int i=0;i<HEIGHT;i++){				
		//if(ball_cnt_x[i]<j_max+30 && ball_cnt_x[i]>j_max-30){
			for(int j=0;j<WIDTH;j++){
				if(!(src_ball.at<Vec3b>(i, j).val[0]==255 && src_ball.at<Vec3b>(i, j).val[1]==255 && src_ball.at<Vec3b>(i, j).val[2]==255)){
					src_ball.at<Vec3b>(i, j).val[0] = 0;
					src_ball.at<Vec3b>(i, j).val[1] = 0;
					src_ball.at<Vec3b>(i, j).val[2] = 0;
				}
			}
		//}
//		ball_cnt_x[i] = 0;
	}
//////////////////////////////////////////////////////////////////////////////////////
		Mat src_line;
		src_ball.copyTo(src_line);

cv::Mat contours;
 cv::Canny(src_line, contours, 125, 350);


  // 선 감지 위한 허프 변환
 std::vector<cv::Vec2f> lines;
 cv::HoughLines(contours, lines, 
     1,PI/180, // 단계별 크기
     80);  // 투표(vote) 최대 개수

  // 선 그리기
 cv::Mat result(contours.rows, contours.cols, CV_8U, cv::Scalar(255));
 std::cout << "Lines detected: " << lines.size() << std::endl;

  // 선 벡터를 반복해 선 그리기
 std::vector<cv::Vec2f>::const_iterator it= lines.begin();
 while (it!=lines.end()) {
  float rho = (*it)[0];   // 첫 번째 요소는 rho 거리
  float theta = (*it)[1]; // 두 번째 요소는 델타 각도
  if (theta < PI/4. || theta > 3.*PI/4.) { // 수직 행
   cv::Point pt1(rho/cos(theta), 0); // 첫 행에서 해당 선의 교차점   
   cv::Point pt2((rho-result.rows*sin(theta))/cos(theta), result.rows);
    // 마지막 행에서 해당 선의 교차점
   cv::line(src_line, pt1, pt2, cv::Scalar(255), 1); // 하얀 선으로 그리기

   } else { // 수평 행
   cv::Point pt1(0,rho/sin(theta)); // 첫 번째 열에서 해당 선의 교차점  
   cv::Point pt2(result.cols,(rho-result.cols*cos(theta))/sin(theta));
    // 마지막 열에서 해당 선의 교차점
   cv::line(src_line, pt1, pt2, cv::Scalar(0,0,255), 1); // 하얀 선으로 그리기
	std::cout << "line: (" << " " << pt1.x << " " << pt1.y << " " << pt2.x << " " << pt2.y << "," << theta*57.3 << ")\n"; 
  }
  //std::cout << "line: (" << pt1.x << "," << theta << ")\n"; 
  ++it;
 }


		imshow("test2", src_line);

//////////////////////////////////////////////////////////////////////////////////////


	    cvtColor(src_ball, src_ball_gray, COLOR_BGR2GRAY );
		
		// Reduce the noise so we avoid false circle detection
	    GaussianBlur(src_ball_gray, src_ball_gray, Size(9, 9), 2, 2 );
		
		cannyThreshold = std::max(cannyThreshold, 1);
        accumulatorThreshold = std::max(accumulatorThreshold, 1);

        //runs the detection, and update the display
        HoughDetection(src_ball_gray, src_ball, cannyThreshold, accumulatorThreshold);
}


void HoughDetection(const Mat& src_gray, const Mat& src_display, int cannyThreshold, int accumulatorThreshold)
    {
        // will hold the results of the detection
        std::vector<Vec3f> circles;
        // runs the actual detection
        HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, cannyThreshold, accumulatorThreshold, 0, 0 );

        // clone the colour, input image for displaying purposes
        Mat display = src_display.clone();
				printf("%d\n", circles.size());

        int max_radius = 0, max_r_i = 0; // added by joon

				for( size_t i = 0; i < circles.size(); i++ )
        {
            int radius = cvRound(circles[i][2]);
        		if(max_radius <= radius){
							max_radius = radius;
							max_r_i = i;		
						}
        }
				
				if(circles.size()>0){
				    Point center(cvRound(circles[max_r_i][0]), cvRound(circles[max_r_i][1]));
            int radius = cvRound(circles[max_r_i][2]);
            // circle centery
            circle( display, center, 3, Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            circle( display, center, radius, Scalar(0,0,255), 3, 8, 0 );
			line(display, Point2f(WIDTH/2, HEIGHT-1), center, Scalar(0, 255, 255), 1);
			if(center.x<WIDTH/2){
				double ball_degree;
				ball_degree = atan((HEIGHT-center.y)/(WIDTH/2-center.x))*57.3;
				cout << "!" << center.y << " " << center.x << endl;
				stringstream ss_ball_degree;
				ss_ball_degree << setprecision(1) << fixed<< ball_degree;
				string str_ball_degree = ss_ball_degree.str();
				putText(display, str_ball_degree, Point2f(WIDTH/2 + 10, HEIGHT-10), FONT_HERSHEY_PLAIN, 1.5, Scalar(255, 255, 255), 1, 8, false);

			}
			else{

			}
			}

        // shows the results
		
        imshow("find_ball", display);
}

void load_settings() {
	fp = fopen("./threshold.txt", "r");
	fscanf(fp, "%d", &lowRedThres);
	fscanf(fp, "%d", &highRedThres);
	fscanf(fp, "%d", &diffRedThres);

	fscanf(fp, "%d", &lowGreenThres);
	fscanf(fp, "%d", &highGreenThres);
	fscanf(fp, "%d", &diffGreenThres);

	fscanf(fp, "%d", &BlackThres);
	fscanf(fp, "%d", &WhiteThres);
	fscanf(fp, "%d", &lowThreshold);
}

void save_settings() {
	fclose(fp);
	fp = fopen("./threshold.txt", "w");
	fprintf(fp, "%d\n", lowRedThres);
	fprintf(fp, "%d\n", highRedThres);
	fprintf(fp, "%d\n", diffRedThres);

	fprintf(fp, "%d\n", lowGreenThres);
	fprintf(fp, "%d\n", highGreenThres);
	fprintf(fp, "%d\n", diffGreenThres);

	fprintf(fp, "%d\n", BlackThres);
	fprintf(fp, "%d\n", WhiteThres);
	fprintf(fp, "%d\n", lowThreshold);

	fclose(fp);
}
