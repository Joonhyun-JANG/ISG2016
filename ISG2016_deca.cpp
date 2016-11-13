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

using namespace cv;
using namespace std;

////// Global variables start //////

int fd;
int milk_x_max=0, milk_y_max=0;
VideoCapture cap(0);

Mat src, src_gray, src_HSV, src_ROI, src_line;;
Mat src_edge, detected_edges;

// for canny detect
int edgeThresh = 1, lowThreshold, ratio = 3, kernel_size = 3;
int const max_lowThreshold = 300; 

// for HSV converting
int lowRedThres = 0, highRedThres = 0, lowGreenThres = 0, highGreenThres = 0;
int diffRedThres = 0, diffGreenThres = 0;

int milk_width_min=0, milk_width_max=0, milk_height_min=0, milk_height_max=0;
int exit_status=0,input_status=0;
char window_Edge[15] = "Edge Map";
char window_HSV[15] = "Hue based";
char window_ROI[15] = "ROI";
int milk_map_front[3][4] = {{9,9,10,10}, {5,6,7,8}, {1,2,3,4}};
int milk_map_down[6][4] = {{16,17,17,18},{13,14,14,15},{10,11,11,12},{7,8,8,9},{4,5,5,6},{1,2,2,3}};

FILE *fp;

////// Global variables end //////

#define WIDTH 320
#define HEIGHT 240
#define PI 3.1415926

int initialize();
void make_windows();
void ThresRefresh(int, void*);
void CannyThreshold(int, void*);
int detect_ball_line();
void load_settings();
void save_settings();

/** @function main */
int main( int argc, char** argv )
{
	int first_frame = 1;
	int look_down = 0;
	if (initialize() == -1) return -1;
	load_settings();
	make_windows();

	while(1){
		bool bSuccess = cap.read(src);
		if (!bSuccess)
		{
			cout << "Cannot read a frame from camera" << endl;
			break;
		}
		if(first_frame==1){
			src_edge.create( src.size(), src.type() );
			src_line.create( src.size(), src.type() );
			src_ROI.create( src.size(), src.type() );
			first_frame = 0;
		}
		// Hue based start
		cvtColor(src, src_HSV, COLOR_BGR2HSV);

		for (int i = 0; i<HEIGHT; i++) {
			for (int j = 0; j<WIDTH; j++) {
				if (src_HSV.at<Vec3b>(i, j).val[0]<lowRedThres || src_HSV.at<Vec3b>(i, j).val[0]>highRedThres) {
					// find RED
					int BG_tmp;
					BG_tmp = (src.at<Vec3b>(i, j).val[0] + src.at<Vec3b>(i, j).val[1]) / 2;
					if ((src.at<Vec3b>(i, j).val[2] - BG_tmp)>diffRedThres) {
						src_HSV.at<Vec3b>(i, j).val[0] = 0;
						src_HSV.at<Vec3b>(i, j).val[1] = 0;
						src_HSV.at<Vec3b>(i, j).val[2] = 255;
						
						src_line.at<Vec3b>(i, j).val[0] = 0;
						src_line.at<Vec3b>(i, j).val[1] = 0;
						src_line.at<Vec3b>(i, j).val[2] = 255;
					}
					else {
						src_HSV.at<Vec3b>(i, j).val[0] = 0;
						src_HSV.at<Vec3b>(i, j).val[1] = 0;
						src_HSV.at<Vec3b>(i, j).val[2] = 0;

						src_line.at<Vec3b>(i, j).val[0] = 0;
						src_line.at<Vec3b>(i, j).val[1] = 0;
						src_line.at<Vec3b>(i, j).val[2] = 0;
					}
				}
				else if (lowGreenThres < src_HSV.at<Vec3b>(i, j).val[0] && src_HSV.at<Vec3b>(i, j).val[0]<highGreenThres) {
					// find Green
					int RB_tmp;
					RB_tmp = (src.at<Vec3b>(i, j).val[0] + src.at<Vec3b>(i, j).val[2]) / 2;
					if ((src.at<Vec3b>(i, j).val[1] - RB_tmp)>diffGreenThres) {
						src_HSV.at<Vec3b>(i, j).val[0] = 0;
						src_HSV.at<Vec3b>(i, j).val[1] = 255;
						src_HSV.at<Vec3b>(i, j).val[2] = 0;
					}
					else {
						src_HSV.at<Vec3b>(i, j).val[0] = 0;
						src_HSV.at<Vec3b>(i, j).val[1] = 0;
						src_HSV.at<Vec3b>(i, j).val[2] = 0;

						src_line.at<Vec3b>(i, j).val[0] = 0;
						src_line.at<Vec3b>(i, j).val[1] = 0;
						src_line.at<Vec3b>(i, j).val[2] = 0;
					}

				}
				else {
					src_HSV.at<Vec3b>(i, j).val[0] = 0;
					src_HSV.at<Vec3b>(i, j).val[1] = 0;
					src_HSV.at<Vec3b>(i, j).val[2] = 0;

					src_line.at<Vec3b>(i, j).val[0] = 0;
					src_line.at<Vec3b>(i, j).val[1] = 0;
					src_line.at<Vec3b>(i, j).val[2] = 0;
				}
			}
		}
		imshow(window_HSV, src_HSV);
		
		
		if(detect_ball_line() == 1){ // line detected
			for(int j=0;j<WIDTH;j++){
				for(int i=0;i<HEIGHT;i++){
					if(src_ROI.at<Vec3b>(i, j).val[0] == 255){
						break;
					}
					else{
						src_ROI.at<Vec3b>(i, j).val[0] = 0;
						src_ROI.at<Vec3b>(i, j).val[1] = 0;
						src_ROI.at<Vec3b>(i, j).val[2] = 0;
					}
				}
			}
		}

	
		cvtColor(src_ROI, src_gray, CV_BGR2GRAY);
		CannyThreshold(0, 0);
		if(look_down==0){
			for(int i=0;i<HEIGHT;i+=80){
				for(int j=0;j<WIDTH;j++){
					src_ROI.at<Vec3b>(i, j).val[0] = 0;
					src_ROI.at<Vec3b>(i, j).val[1] = 255;
					src_ROI.at<Vec3b>(i, j).val[2] = 255;
				}
			}

			for(int i=80;i<HEIGHT;i++){
				for(int j=0;j<WIDTH;j+=80){
					src_ROI.at<Vec3b>(i, j).val[0] = 0;
					src_ROI.at<Vec3b>(i, j).val[1] = 255;
					src_ROI.at<Vec3b>(i, j).val[2] = 255;
				}
			}
			for(int i=0;i<80;i++){
				for(int j=0;j<WIDTH;j+=107){
					src_ROI.at<Vec3b>(i, j).val[0] = 0;
					src_ROI.at<Vec3b>(i, j).val[1] = 255;
					src_ROI.at<Vec3b>(i, j).val[2] = 255;
				}
			}
		}
		else{
			for(int i=0;i<HEIGHT;i+=40){
				for(int j=0;j<WIDTH;j++){
					src_ROI.at<Vec3b>(i, j).val[0] = 0;
					src_ROI.at<Vec3b>(i, j).val[1] = 0;
					src_ROI.at<Vec3b>(i, j).val[2] = 255;
				}
			}

			for(int i=0;i<HEIGHT;i++){
				src_ROI.at<Vec3b>(i, 80).val[0] = 0;
				src_ROI.at<Vec3b>(i, 80).val[1] = 0;
				src_ROI.at<Vec3b>(i, 80).val[2] = 255;

				src_ROI.at<Vec3b>(i, WIDTH-80).val[0] = 0;
				src_ROI.at<Vec3b>(i, WIDTH-80).val[1] = 0;
				src_ROI.at<Vec3b>(i, WIDTH-80).val[2] = 255;
			}
		}
		
		//cout << milk_x_max/80 << " " << milk_y_max/80 << endl;
		imshow(window_ROI, src_ROI);
		
		imshow(window_Edge, src_edge);

		if(exit_status==1) break;
		if(input_status==1){
			int motion_num;
			printf("input: ");
			scanf("%d", &motion_num);
			input_status=0;
			fflush (stdout) ;
			serialPutchar (fd, (unsigned char)motion_num);
			while (serialDataAvail(fd))
			{
				printf(" -> %3d\n", serialGetchar(fd));
				fflush (stdout) ;
			}
			make_windows();
		}
		while (serialDataAvail(fd))
		{
				int tmp;
				printf(" robot: %3d\n", tmp=serialGetchar(fd));
				switch(tmp){
					case 29: look_down = 0; break;
					case 31: look_down = 1; break;
					case 96: printf("Where is milk? "); 
						if(milk_y_max>0 && milk_x_max>0) {
							if(look_down == 0){
								int milk_finded=milk_map_front[milk_y_max/80][milk_x_max/80];
								serialPutchar (fd, (unsigned char)milk_finded); 
								printf(" -> %3d\n", milk_finded);
							}
							else if(look_down == 1){

							}
						}
						else{
							printf("Not founded\n");
							serialPutchar (fd, (unsigned char)0); 
						}
						//printf("a");
						break;
					case 97: printf("Look down\n"); 
						look_down = 1;
						if(milk_y_max>0 && milk_x_max>0) {
							int milk_finded=milk_map_front[milk_y_max/80][milk_x_max/80];
							serialPutchar (fd, (unsigned char)milk_finded); 
							printf(" -> %3d\n", milk_finded);
						}
						else{
							printf("Not founded\n");
							serialPutchar (fd, (unsigned char)38); 
						}
						//printf("a");
						break;

					case 105: printf("ISG19 milk_horizon(2,5)");
						if(milk_y_max>0 && milk_x_max>0) {
							int milk_finded=milk_map_down[milk_y_max/40][milk_x_max/80];
							if(milk_finded==2 || milk_finded==5){
								printf(" -> %3d\n", milk_finded);
								serialPutchar (fd, (unsigned char)1); 	
							}
							else{
								printf(" -> Not founded\n");
								serialPutchar (fd, (unsigned char)0);
							}
						}
						else{
								printf(" -> Not founded\n");
								serialPutchar (fd, (unsigned char)0);
						}
						look_down = 1;
						break; 

					case 106: printf("ISG19 milk_horizon(1~6)");
						if(milk_y_max>0 && milk_x_max>0) {
							printf(" -> %3d\n", 6-(milk_y_max/40));
							serialPutchar (fd, (unsigned char)6-(milk_y_max/40));
						}
						else{
							printf(" -> Not founded\n");
							serialPutchar (fd, (unsigned char)0); 
						}
						look_down = 1;
						break; 
				}
				fflush (stdout) ;
		}

		// Wait until user exit program by pressing a key
		waitKey(30);

		}
	save_settings();
	return 0;
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
	// Create a "Edge window"
	namedWindow(window_Edge, CV_WINDOW_AUTOSIZE);
	createTrackbar( "Min Threshold:", window_Edge, &lowThreshold, max_lowThreshold, ThresRefresh );
	createTrackbar( "INPUT", window_Edge, &input_status, 1, ThresRefresh );
	createTrackbar( "EXIT", window_Edge, &exit_status, 1, ThresRefresh );

	// Create a "HSV window"
	namedWindow(window_HSV, CV_WINDOW_AUTOSIZE);
	createTrackbar("Red Min:", window_HSV, &lowRedThres, 255, ThresRefresh);
	createTrackbar("Red Max:", window_HSV, &highRedThres, 255, ThresRefresh);
	createTrackbar("Red Diff:", window_HSV, &diffRedThres, 255, ThresRefresh);
	createTrackbar("Green Min:", window_HSV, &lowGreenThres, 255, ThresRefresh);
	createTrackbar("Green Max:", window_HSV, &highGreenThres, 255, ThresRefresh);
	createTrackbar("Green Diff:", window_HSV, &diffGreenThres, 255, ThresRefresh);	

	namedWindow(window_ROI, CV_WINDOW_AUTOSIZE);
	createTrackbar("width_min:", window_ROI, &milk_width_min, 320, ThresRefresh);
	createTrackbar("width_max:", window_ROI, &milk_width_max, 320, ThresRefresh);
	createTrackbar("height_min:", window_ROI, &milk_height_min, 240, ThresRefresh);
	createTrackbar("height_max:", window_ROI, &milk_height_max, 240, ThresRefresh);

}

void ThresRefresh(int, void*){

}

void CannyThreshold(int, void*)
{

  /// Reduce noise with a kernel 3x3
	blur( src_gray, detected_edges, Size(3,3) );

  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  /// Using Canny's output as a mask, we display our result
  src_edge = Scalar::all(0);
  detected_edges.copyTo(src_edge);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	int refinery_count = 0;

	findContours(detected_edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point());
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());
	vector<Rect> boundRect2(contours.size());

	milk_x_max=0, milk_y_max=0;
	for(int i = 0; i<contours.size(); i++){
		approxPolyDP(Mat(contours[i]), contours_poly[i], 1, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
		if(boundRect[i].height > milk_height_min && boundRect[i].height < milk_height_max && boundRect[i].width > milk_width_min && boundRect[i].width < milk_width_max){
			int milk_G_cnt=0;
			for(int k=boundRect[i].tl().y; k<boundRect[i].br().y; k++){
				for(int l=boundRect[i].tl().x; l<boundRect[i].br().x; l++){
					if(src_HSV.at<Vec3b>(k, l).val[1] == 255){
						milk_G_cnt++;
					}
				}
			}
			if(milk_G_cnt>20){
				if(milk_y_max<boundRect[i].br().y){
					milk_y_max = boundRect[i].br().y;
					milk_x_max = (boundRect[i].tl().x+boundRect[i].br().x)/2;
				}
				rectangle(src_ROI, boundRect[i].tl(), boundRect[i].br(), Scalar(128,0,128), 1, 8, 0);
				stringstream ss_milk;
				ss_milk << milk_G_cnt;	
				string str_milk = ss_milk.str();
				putText(src_ROI, str_milk, boundRect[i].tl(), FONT_HERSHEY_PLAIN, 0.7, Scalar(128, 0, 255), 1, 8, false);
			}
		}
		
	}

/*
	for(int i = 0; i<contours.size(); i++){
		ratio = (double) boundRect[i].height / boundRect[i].width;
		
		if((ratio <= 2.0) && (ratio >= 1.3) && (boundRect[i].area() <= 2000) && (boundRect[i].area() >= 200)){
			drawContours(src_edge, contours, i, Scalar(0,255,255), 1, 8, hierarchy, 0, Point());
			rectangle(src_edge, boundRect[i].tl(), boundRect[i].br(), Scalar(128,0,128), 1, 8, 0);

			refinery_count += 1;
			boundRect2[refinery_count] = boundRect[i];
		}
	}
*/
	boundRect2.resize(refinery_count);


 }



int detect_ball_line(){

//////////////////////////////////////////////////////////////////////////////////////
		int detect_line = 0;
		

		cv::Mat contours;
		cv::Canny(src_line, contours, 125, 350);

		// 선 감지 위한 허프 변환
		std::vector<cv::Vec2f> lines;
		cv::HoughLines(contours, lines, 1,PI/180, // 단계별 크기
					     80);  // 투표(vote) 최대 개수
		
		// 선 그리기
		cv::Mat result(contours.rows, contours.cols, CV_8U, cv::Scalar(255));
		//std::cout << "Lines detected: " << lcopyToines.size() << std::endl;

		// 선 벡터를 반복해 선 그리기
		std::vector<cv::Vec2f>::const_iterator it= lines.begin();
		float rho_avg=0, theta_avg=0; 
		while (it!=lines.end()) {
			float rho = (*it)[0];   // 첫 번째 요소는 rho 거리
			float theta = (*it)[1]; // 두 번째 요소는 델타 각도
			if (theta < PI/4. || theta > 3.*PI/4.) { // 수직 행
				Point pt1(rho/cos(theta), 0); // 첫 행에서 해당 선의 교차점   
				Point pt2((rho-result.rows*sin(theta))/cos(theta), result.rows);
				// 마지막 행에서 해당 선의 교차점
				line(src_line, pt1, pt2, cv::Scalar(0,255,255), 1); // 하얀 선으로 그리기
			} 
			else { // 수평 행
				rho_avg += rho;
				theta_avg += theta;
				Point pt1(0,rho/sin(theta)); // 첫 번째 열에서 해당 선의 교차점  
				Point pt2(result.cols,(rho-result.cols*cos(theta))/sin(theta));
				// 마지막 열에서 해당 선의 교차점
				line(src_line, pt1, pt2, cv::Scalar(0,255,255), 1); // 하얀 선으로 그리기
			}
			++it;
		}
		
		if(lines.size()!=0){
			rho_avg /= lines.size();
			theta_avg /= lines.size();
			float pt1_y, pt2_y;

			pt1_y = rho_avg/sin(theta_avg);
			pt2_y = (rho_avg-result.cols*cos(theta_avg))/sin(theta_avg);
			Point pt1_avg(0, pt1_y); // 첫 번째 열에서 해당 선의 교차점  
			Point pt2_avg(result.cols, pt2_y);

			// 마지막 열에서 해당 선의 교차점
			line(src_line, pt1_avg, pt2_avg, cv::Scalar(255,0,0), 1); // 하얀 선으로 그리기
			src.copyTo(src_ROI);
			line(src_ROI, pt1_avg, pt2_avg, cv::Scalar(255,0,0), 1); // 하얀 선으로 그리기
			detect_line = 1;

			theta_avg = 90-(theta_avg*57.3);
			if(-5<theta_avg && theta_avg<5){
				int pt_mid = (pt1_y+pt2_y)/2;
				line(src_line, Point(160,319), Point(160,pt_mid), cv::Scalar(128,0,255), 1);
				stringstream ss_Line_distance;
				ss_Line_distance << "Distance: " << HEIGHT-pt_mid;	
				string str_Line_distance = ss_Line_distance.str();
				putText(src_line, str_Line_distance, Point2f(70, 220), FONT_HERSHEY_PLAIN, 0.7, Scalar(128, 0, 255), 1, 8, false);
			}
			stringstream ss_Line_degree;
			ss_Line_degree << "Degree: " << theta_avg;
			string str_Line_degree = ss_Line_degree.str();
			putText(src_line, str_Line_degree, Point2f(180, 235), FONT_HERSHEY_PLAIN, 0.7, Scalar(255, 255, 255), 1, 8, false);
		}
		else{
			src.copyTo(src_ROI);
			theta_avg = 0;
			putText(src_line, "Not detected", Point2f(180, 235), FONT_HERSHEY_PLAIN, 0.7, Scalar(0, 0, 255), 1, 8, false);
		}
		stringstream ss_Line;
		ss_Line << "Lines detected: " << lines.size();
		string str_Line = ss_Line.str();
		putText(src_line, str_Line, Point2f(180, 220), FONT_HERSHEY_PLAIN, 0.7, Scalar(255, 255, 255), 1, 8, false);
		
		

		imshow("Line_find", src_line);

//////////////////////////////////////////////////////////////////////////////////////

/*
	    cvtColor(src_ball, src_ball_gray, COLOR_BGR2GRAY );
		
		// Reduce the noise so we avoid false circle detection
	    GaussianBlur(src_ball_gray, src_ball_gray, Size(9, 9), 2, 2 );
		
		cannyThreshold = std::max(cannyThreshold, 1);
        accumulatorThreshold = std::max(accumulatorThreshold, 1);

        //runs the detection, and update the display
        HoughDetection(src_ball_gray, src_ball, cannyThreshold, accumulatorThreshold);
*/

	return detect_line;
}


void load_settings() {
	fp = fopen("./threshold.txt", "r");
	fscanf(fp, "%d", &lowRedThres);
	fscanf(fp, "%d", &highRedThres);
	fscanf(fp, "%d", &diffRedThres);

	fscanf(fp, "%d", &lowGreenThres);
	fscanf(fp, "%d", &highGreenThres);
	fscanf(fp, "%d", &diffGreenThres);

	fscanf(fp, "%d", &lowThreshold);

	fscanf(fp, "%d", &milk_width_min);
	fscanf(fp, "%d", &milk_width_max);
	fscanf(fp, "%d", &milk_height_min);
	fscanf(fp, "%d", &milk_height_max);

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

	fprintf(fp, "%d\n", lowThreshold);

	fprintf(fp, "%d\n", milk_width_min);
	fprintf(fp, "%d\n", milk_width_max);
	fprintf(fp, "%d\n", milk_height_min);
	fprintf(fp, "%d\n", milk_height_max);

	fclose(fp);
}


