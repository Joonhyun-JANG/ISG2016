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
int look_down, test_mode=0;

VideoCapture cap(0);

Mat src, src_gray, src_HSV, src_ROI, src_line;
Mat src_edge, detected_edges, src_status;

// for canny detect
int edgeThresh = 1, lowThreshold, ratio = 3, kernel_size = 3;
int const max_lowThreshold = 300; 

// for HSV converting
int lowRedThres = 0, highRedThres = 0, lowGreenThres = 0, highGreenThres = 0;
int diffRedThres = 0, diffGreenThres = 0;
int saturationThres = 0;

// for line detecting
int lineThreshold=0, voteThreshold=80;
int line_distance_front[270]={0,};

int pt_mid=0;


int milk_width_min=0, milk_width_max=0, milk_height_min=0, milk_height_max=0;
int exit_status=0,input_status=0;
char window_Edge[15] = "Edge Map";
char window_HSV[15] = "Hue based";
char window_ROI[15] = "ROI";
char window_Line[15] = "line find";
char window_src[15] = "Original";
char window_status[15] = "STATUS";

int milk_map_front[3][4] = {{9,9,10,10}, {5,6,7,8}, {1,2,3,4}};
int milk_map_down[6][4] = {{16,17,17,18},{13,14,14,15},{10,11,11,12},{7,8,8,9},{4,5,5,6},{1,2,2,3}};

float theta_avg=0, theta_avg2=0;
float mid_theta = 0;

float fps_tmp=0;

FILE *fp;

////// Global variables end //////

#define WIDTH 320
#define HEIGHT 240
#define PI 3.1415926

typedef unsigned long DWORD;
typedef unsigned long int tick32_t;

static DWORD lastTime = 0;
tick32_t get_tick_count(){\
	tick32_t tick=0ul;
	struct timespec tp;
	clock_gettime(CLOCK_MONOTONIC, &tp);
	tick = (tp.tv_sec*1000ul) + (tp.tv_nsec/1000ul/1000ul);
	return tick;
}



////// functions definition //////

int initialize();
void make_windows();
void ThresRefresh(int, void*);
void CannyThreshold(int, void*);
int detect_ball_line();
void load_settings();
void save_settings();
void filter_milk_and_line();
void delete_outofline(int line_YN);
void UpdateFPS();


/** @function main */
int main( int argc, char** argv )
{
	int first_frame = 1;
	int command_cmd=0;
	look_down = 0;

	if (initialize() == -1) return -1;
	load_settings();
	
	make_windows();
	lastTime = get_tick_count();

	while(1){
		
		if(command_cmd>9999) command_cmd=0;
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
			src_HSV.create( src.size(), src.type() );
			src_status.create( src.size(), src.type() );
			first_frame = 0;
		}

		src_status = Scalar::all(0);
		UpdateFPS();
		imshow(window_src, src);
		src.copyTo(src_ROI);
///////test code start ///////////	
		if(test_mode==1){
		filter_milk_and_line();
		delete_outofline(detect_ball_line());
		CannyThreshold(0, 0);

		}
///////test code end   ///////////

		while (serialDataAvail(fd))
		{
				int tmp;
				printf("@%5d@ robot: %3d  ", command_cmd++, tmp=serialGetchar(fd));
				
				switch(tmp){
					case 1: printf("received1\n"); break;
					case 2: printf("received2\n"); break;
					case 29: look_down = 0; break;
					case 31: look_down = 1; break;
					case 95: printf("find milk(30~150) "); 
						src_ROI = Scalar::all(0);
						filter_milk_and_line();
						CannyThreshold(0, 0);
						if(milk_x_max>107 && milk_x_max<214){
							serialPutchar (fd, (unsigned char)1); 
							printf(" -> yes\n");
						}
						else{
							serialPutchar (fd, (unsigned char)0); 
							printf(" -> no\n");
						}	
						break;
					case 96: printf("Where is milk? "); 
						src_ROI = Scalar::all(0);
						filter_milk_and_line();
						CannyThreshold(0, 0);
						if(milk_y_max>0 && milk_x_max>0) {
							if(look_down == 0){
								int milk_finded;
								if(milk_y_max>80){
									milk_finded=milk_map_front[milk_y_max/80][milk_x_max/80];
								}
								else{
									milk_finded=9+(milk_x_max/107);
								}
								serialPutchar (fd, (unsigned char)milk_finded); 
								printf(" -> %3d(front)\n", milk_finded);
							}
							else if(look_down == 1){
								int milk_finded=milk_map_down[milk_y_max/40][milk_x_max/80];
								serialPutchar (fd, (unsigned char)milk_finded); 
								printf(" -> %3d(down)\n", milk_finded);
							}
						}
						else{
							printf("Not founded\n");
							src_ROI = Scalar::all(0);
							serialPutchar (fd, (unsigned char)0); 
						}
						//printf("a");
						break;
					case 97: printf("Look down\n"); 
						look_down = 1;
						break;
					case 98: printf("Line detect: "); 

						filter_milk_and_line();
						if(detect_ball_line() == 1){ // line detected
							
							
							int degree_data=0;
							if(-90<=mid_theta && mid_theta<-75){
								degree_data=1;
							}
							else if(-75<=mid_theta && mid_theta<-65){
								degree_data=2;
							}
else if(-65<=mid_theta && mid_theta<-65){
								degree_data=3;
							}
else if(-55<=mid_theta && mid_theta<-45){
								degree_data=4;
							}
else if(-45<=mid_theta && mid_theta<-35){
								degree_data=5;
							}
else if(-35<=mid_theta && mid_theta<-25){
								degree_data=6;
							}
else if(-25<=mid_theta && mid_theta<-15){
								degree_data=7;
							}
else if(-15<=mid_theta && mid_theta<-5){
								degree_data=8;
							}
else if(-5<=mid_theta && mid_theta<5){
								degree_data=9;
							}
else if(5<=mid_theta && mid_theta<15){
								degree_data=10;
							}
else if(15<=mid_theta && mid_theta<25){
								degree_data=11;
							}
else if(25<=mid_theta && mid_theta<35){
								degree_data=12;
							}
else if(35<=mid_theta && mid_theta<45){
								degree_data=13;
							}
else if(45<=mid_theta && mid_theta<55){
								degree_data=14;
							}
else if(55<=mid_theta && mid_theta<65){
								degree_data=15;
							}
else if(65<=mid_theta && mid_theta<75){
								degree_data=16;
							}
else if(75<=mid_theta && mid_theta<90){
								degree_data=17;
							}
							// send degree
							printf("%f (%d)\n", mid_theta, degree_data);
							serialPutchar (fd, (unsigned char)192+degree_data); 
						}
						else{
							printf("Not detected\n");
							serialPutchar (fd, (unsigned char)128); 
						}
						break;


					case 99: printf("Look front\n"); 
						look_down = 0;
						break;

					case 101: printf("Line distance : %d\n", (HEIGHT-pt_mid)/40); 
						serialPutchar (fd, (unsigned char)(HEIGHT-pt_mid)/40); 
						break;
					case 105: printf("ISG19 milk_horizon(2,5)");
						src_ROI = Scalar::all(0);
						filter_milk_and_line();
						CannyThreshold(0, 0);
						if(milk_y_max>0 && milk_x_max>0) {
							int milk_finded=milk_map_down[milk_y_max/40][milk_x_max/80];
							if(80<milk_x_max && milk_x_max<240){
								printf(" -> %3d\n", milk_finded);
								serialPutchar (fd, (unsigned char)milk_finded); 	
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
						src_ROI = Scalar::all(0);
						filter_milk_and_line();
						CannyThreshold(0, 0);
						if(milk_y_max>0 && milk_x_max>0) {
							printf(" -> %3d\n", 6-(int)(milk_y_max/40));
							serialPutchar (fd, (unsigned char)6-(int)(milk_y_max/40));
						}
						else{
							printf(" -> Not founded\n");
							serialPutchar (fd, (unsigned char)0); 
							src_ROI = Scalar::all(0);
						}
						look_down = 1;
						break; 
				}
				fflush (stdout) ;
		}
		//current time-pre time;

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
			for(int i=0;i<HEIGHT;i++){
				for(int j=107;j<WIDTH;j+=107){
					src_ROI.at<Vec3b>(i, j).val[0] = 0;
					src_ROI.at<Vec3b>(i, j).val[1] = 0;
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
		imshow(window_HSV, src_HSV);
		imshow(window_Edge, src_edge);
		imshow(window_status, src_status);

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
		

		
		// Wait until user exit program by pressing a key
		waitKey(5);
		//previous time = present time save

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
	createTrackbar( "TEST", window_Edge, &test_mode, 1, ThresRefresh );
	createTrackbar( "EXIT", window_Edge, &exit_status, 1, ThresRefresh );

	// Create a "HSV window"
	namedWindow(window_HSV, CV_WINDOW_AUTOSIZE);
	createTrackbar("Red Min:", window_HSV, &lowRedThres, 255, ThresRefresh);
	createTrackbar("Red Max:", window_HSV, &highRedThres, 255, ThresRefresh);
	createTrackbar("Red Diff:", window_HSV, &diffRedThres, 255, ThresRefresh);
	createTrackbar("Green Min:", window_HSV, &lowGreenThres, 255, ThresRefresh);
	createTrackbar("Green Max:", window_HSV, &highGreenThres, 255, ThresRefresh);
	createTrackbar("Green Diff:", window_HSV, &diffGreenThres, 255, ThresRefresh);	
	createTrackbar("S Diff:", window_HSV, &saturationThres, 255, ThresRefresh);	

	// Create a "ROI window"
	namedWindow(window_ROI, CV_WINDOW_AUTOSIZE);
	createTrackbar("width_min:", window_ROI, &milk_width_min, 320, ThresRefresh);
	createTrackbar("width_max:", window_ROI, &milk_width_max, 320, ThresRefresh);
	createTrackbar("height_min:", window_ROI, &milk_height_min, 240, ThresRefresh);
	createTrackbar("height_max:", window_ROI, &milk_height_max, 240, ThresRefresh);
	
	createTrackbar("width_min2:", window_ROI, &milk_width_min, 320, ThresRefresh);
	createTrackbar("width_max2:", window_ROI, &milk_width_max, 320, ThresRefresh);
	createTrackbar("height_min2:", window_ROI, &milk_height_min, 240, ThresRefresh);
	createTrackbar("height_max2:", window_ROI, &milk_height_max, 240, ThresRefresh);
	
	createTrackbar("width_min3:", window_ROI, &milk_width_min, 320, ThresRefresh);
	createTrackbar("width_max3:", window_ROI, &milk_width_max, 320, ThresRefresh);
	createTrackbar("height_min3:", window_ROI, &milk_height_min, 240, ThresRefresh);
	createTrackbar("height_max3:", window_ROI, &milk_height_max, 240, ThresRefresh);

	// Create a "Line window"
	namedWindow(window_Line, CV_WINDOW_AUTOSIZE);
	createTrackbar("line_thres:", window_Line, &lineThreshold, 320, ThresRefresh);
	createTrackbar("vote_thres:", window_Line, &voteThreshold, 320, ThresRefresh);

	namedWindow(window_src, CV_WINDOW_AUTOSIZE);
}

void ThresRefresh(int, void*){

}

void filter_milk_and_line(){
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
					if ((src.at<Vec3b>(i, j).val[1] - RB_tmp)>diffGreenThres && src_HSV.at<Vec3b>(i, j).val[0]>saturationThres ) {
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

}
void CannyThreshold(int, void*)
{
	

	cvtColor(src, src_gray, CV_BGR2GRAY);
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
				ss_milk << boundRect[i].br().x-boundRect[i].tl().x << " " << boundRect[i].br().y-boundRect[i].tl().y;	
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
	//boundRect2.resize(refinery_count);


 }



int detect_ball_line(){

//////////////////////////////////////////////////////////////////////////////////////
		int detect_line = 0;
		int group1_cnt=0, group2_cnt=0;
		float group1_theta[3]={999,999,999}, group2_theta[3]={999,999,999};
		mid_theta = 0; // real mid line
		int mid_theta_group = 0;
		theta_avg = 0, theta_avg2 = 0;

		cv::Mat contours;
		cv::Canny(src_line, contours, lineThreshold, lineThreshold*ratio);

		// 선 감지 위한 허프 변환
		std::vector<cv::Vec2f> lines;
		cv::HoughLines(contours, lines, 1,PI/180, // 단계별 크기
					    voteThreshold);  // 투표(vote) 최대 개수
		
		// 선 그리기
		cv::Mat result(contours.rows, contours.cols, CV_8U, cv::Scalar(255));
		//std::cout << "Lines detected: " << lcopyToines.size() << std::endl;

		// 선 벡터를 반복해 선 그리기
		std::vector<cv::Vec2f>::const_iterator it= lines.begin();
		float rho_avg=0, rho_avg2=0; 

		while (it!=lines.end()) {
			float rho = (*it)[0];   // 첫 번째 요소는 rho 거리
			float theta = (*it)[1]; // 두 번째 요소는 델타 각도
			float theta_tmp = theta*57.3;
			int group = 0;

			if(group==0 && group1_theta[0]==999){
					group1_theta[0] = theta*57.3;
					group1_theta[1] = group1_theta[0]-15;
					group1_theta[2] = group1_theta[0]+15;
			}
			if(group1_theta[1]<0){
				if((theta_tmp>group1_theta[1]+180) || (group1_theta[2]>theta_tmp)){
					group = 1;
				}
			}
			else if(group1_theta[2]>180){
				if((group1_theta[1]<theta_tmp) || (group1_theta[2]-180)>theta_tmp){
					group = 1;
				}
			}
			else{
				if(group1_theta[1]<theta_tmp && theta_tmp<group1_theta[2]){
					group = 1;
				}
			}

			if(group==0){
					if(group2_theta[0]==999){
						group2_theta[0] = theta*57.3;
						group2_theta[1] = group2_theta[0]-15;
						group2_theta[2] = group2_theta[0]+15;
					}
					if(group2_theta[1]<0){
						if((theta_tmp>group2_theta[1]+180) || (group2_theta[2]>theta_tmp)){
							group = 2;
						}
					}
					else if(group2_theta[2]>180){
						if((group2_theta[1]<theta_tmp) || (group2_theta[2]-180)>theta_tmp){
							group = 2;
						}
					}		
					else{
						if(group2_theta[1]<theta_tmp && theta_tmp<group2_theta[2]){
							group = 2;
						}
					}
			}

			

			


			if (theta < PI/4. || theta > 3.*PI/4.) { // 수직 행
					Point pt1(rho/cos(theta), 0); // 첫 행에서 해당 선의 교차점   
					Point pt2((rho-result.rows*sin(theta))/cos(theta), result.rows);
					// 마지막 행에서 해당 선의 교차점
					if(group==1){
						line(src_line, pt1, pt2, cv::Scalar(0,255,0), 1); // green line
						rho_avg += rho;
						theta_avg += theta;
						group1_cnt++;
					}
					else if(group==2){
						line(src_line, pt1, pt2, cv::Scalar(0,255, 255), 1); // yellow line
						rho_avg2 += rho;
						theta_avg2 += theta;
						group2_cnt++;
					}
			} 
			else { // 수평 행
				Point pt1(0,rho/sin(theta)); // 첫 번째 열에서 해당 선의 교차점  
				Point pt2(result.cols,(rho-result.cols*cos(theta))/sin(theta));
				// 마지막 열에서 해당 선의 교차점

				if(group==1){
						line(src_line, pt1, pt2, cv::Scalar(0,255,0), 1); // green line
						rho_avg += rho;
						theta_avg += theta;
						group1_cnt++;
				}
				else if(group==2){
						line(src_line, pt1, pt2, cv::Scalar(0,255, 255), 1); // yellow line
						rho_avg2 += rho;
						theta_avg2 += theta;
						group2_cnt++;
				}
				
			}
			++it;
		}
		
		if(lines.size()!=0){ // draw median line
			int mid_group = 1;
			/*if(group2_cnt>0){
				float group1_degree=0, group2_degree=0;
				group1_degree = 90-(theta_avg/group1_cnt)*57.3;
				if(group1_degree<0) group1_degree = group1_degree*(-1);
				group2_degree = 90-(theta_avg/group1_cnt)*57.3;
				if(group2_degree<0) group2_degree = group2_degree*(-1);

				if(group2_degree<group1_degree) mid_group=2;
			}*/
	
			float pt1_xy, pt2_xy; // group 1
			float pt3_xy, pt4_xy; // group 2

			if(group1_cnt>0){
				rho_avg /= group1_cnt;
				theta_avg /= group1_cnt;
				
				if (theta_avg < PI/4. || theta_avg > 3.*PI/4.) { // 수직 행
					pt1_xy = rho_avg/cos(theta_avg);
					pt2_xy = (rho_avg-result.rows*sin(theta_avg))/cos(theta_avg);

					Point pt1(pt1_xy, 0); // 첫 행에서 해당 선의 교차점   
					Point pt2(pt2_xy, result.rows);
					line(src_ROI, pt1, pt2, cv::Scalar(255,0,0), 1); // blue line
				}
				else{
					pt1_xy = rho_avg/sin(theta_avg);
					pt2_xy = (rho_avg-result.cols*cos(theta_avg))/sin(theta_avg);

					Point pt1(0,pt1_xy); // 첫 번째 열에서 해당 선의 교차점  
					Point pt2(result.cols,pt2_xy);
					line(src_ROI, pt1, pt2, cv::Scalar(255,0,0), 1); // blue line
				}				
			}
			mid_theta = theta_avg; // group1 is mid
			mid_theta_group = 1;
			
			if(group2_cnt>0){
				rho_avg2 /= group2_cnt;
				theta_avg2 /= group2_cnt;
				
				if (theta_avg2 < PI/4. || theta_avg2 > 3.*PI/4.) { // 수직 행
					pt3_xy = rho_avg2/cos(theta_avg2);
					pt4_xy = (rho_avg2-result.rows*sin(theta_avg2))/cos(theta_avg2);

					Point pt1(pt3_xy, 0); // 첫 행에서 해당 선의 교차점   
					Point pt2(pt4_xy, result.rows);
					line(src_ROI, pt1, pt2, cv::Scalar(255,0,0), 1); // blue line
				}
				else{
					pt3_xy = rho_avg2/sin(theta_avg2);
					pt4_xy = (rho_avg2-result.cols*cos(theta_avg2))/sin(theta_avg2);

					Point pt1(0,pt3_xy); // 첫 번째 열에서 해당 선의 교차점  
					Point pt2(result.cols,pt4_xy);
					line(src_ROI, pt1, pt2, cv::Scalar(255,0,0), 1); // blue line
				}
				if(abs(90-(theta_avg*57.3))>abs(90-(theta_avg2*57.3))){
					mid_theta = theta_avg2;
					mid_theta_group = 2; // group2 is mid
				}
			}
			
			
			detect_line = 1;

			mid_theta = 90-(mid_theta*57.3);
			
			if(-5<mid_theta && mid_theta<5){
				pt_mid;
				if(mid_theta_group == 1) pt_mid = (pt1_xy+pt2_xy)/2;
				else if(mid_theta_group == 2) pt_mid = (pt1_xy+pt2_xy)/2;
				line(src_line, Point(160,319), Point(160,pt_mid), cv::Scalar(128,0,255), 1);
				stringstream ss_Line_distance;
				if(look_down==0) ss_Line_distance << "Distance: " << line_distance_front[HEIGHT-pt_mid] << "cm (" << HEIGHT-pt_mid << ")";	
				string str_Line_distance = ss_Line_distance.str();
				putText(src_line, str_Line_distance, Point2f(10, 220), FONT_HERSHEY_PLAIN, 0.7, Scalar(255, 255, 255), 1, 8, false);
				putText(src_status, str_Line_distance, Point2f(10, 220), FONT_HERSHEY_PLAIN, 0.7, Scalar(255, 255, 255), 1, 8, false);
			}
			
			stringstream ss_Line_degree;
			ss_Line_degree << "Degree: " << mid_theta;
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
		
		

		imshow(window_Line, src_line);

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
	char tmp[25];
	int tmp_int;

	fp = fopen("./threshold.txt", "r");
	fscanf(fp, "%s %d", tmp, &lowRedThres);
	fscanf(fp, "%s %d", tmp, &highRedThres);
	fscanf(fp, "%s %d", tmp, &diffRedThres);

	fscanf(fp, "%s %d", tmp, &lowGreenThres);
	fscanf(fp, "%s %d", tmp, &highGreenThres);
	fscanf(fp, "%s %d", tmp, &diffGreenThres);

	fscanf(fp, "%s %d", tmp, &lowThreshold);

	fscanf(fp, "%s %d", tmp, &milk_width_min);
	fscanf(fp, "%s %d", tmp, &milk_width_max);
	fscanf(fp, "%s %d", tmp, &milk_height_min);
	fscanf(fp, "%s %d", tmp, &milk_height_max);

	fclose(fp);

	fp = fopen("./line_distance_front.txt", "r");
	for(int i=0;i<270;i++){
		fscanf(fp, "%d %d", &tmp_int, &line_distance_front[i]); 
	}
	printf("load parameter\n");
	fclose(fp);
}

void save_settings() {
	
	fp = fopen("./threshold.txt", "w");
	fprintf(fp, "lowRedThres %d\n", lowRedThres);
	fprintf(fp, "highRedThres %d\n", highRedThres);
	fprintf(fp, "diffRedThres %d\n", diffRedThres);

	fprintf(fp, "lowGreenThres %d\n", lowGreenThres);
	fprintf(fp, "highGreenThres %d\n", highGreenThres);
	fprintf(fp, "diffGreenThres %d\n", diffGreenThres);

	fprintf(fp, "lowThreshold %d\n", lowThreshold);

	fprintf(fp, "milk_width_min %d\n", milk_width_min);
	fprintf(fp, "milk_width_max %d\n", milk_width_max);
	fprintf(fp, "milk_height_min %d\n", milk_height_min);
	fprintf(fp, "milk_height_max %d\n", milk_height_max);

	fclose(fp);
}

void delete_outofline(int line_YN){
	if(line_YN){
		int line_start=0;	
		for(int j=0;j<WIDTH;j++){
			int i_tmp=0;
			for(int i=0;i<HEIGHT;i++){	// delete out of line
				if(src_ROI.at<Vec3b>(i, j).val[0] == 255 && src_ROI.at<Vec3b>(i, j).val[1] == 0){
					i_tmp = i;
					if(line_start==0) line_start=1;
					break;
				}		
			}

			if(i_tmp==0 && line_start==1) line_start=2;

			if(i_tmp==0 && line_start==0){ // before line
				if(theta_avg>0){
					for(int i=0;i<HEIGHT;i++){
						
					}
				}

			}
	
			if(i_tmp==0 && line_start==2){ // line ended
				if(theta_avg<0){
					for(int i=0;i<HEIGHT;i++){
						src_ROI.at<Vec3b>(i, j).val[0] = 0;
						src_ROI.at<Vec3b>(i, j).val[1] = 0;
						src_ROI.at<Vec3b>(i, j).val[2] = 0;
					}
				}
			}
			
			if(i_tmp!=0){ // line inside 
				for(int i=0;i<i_tmp;i++){
					src_ROI.at<Vec3b>(i, j).val[0] = 0;
					src_ROI.at<Vec3b>(i, j).val[1] = 0;
					src_ROI.at<Vec3b>(i, j).val[2] = 0;
				}
			}
		}

	} //end if
}



void UpdateFPS(){

	static DWORD frameCount = 0;
	static float timeElapsed = 0.0f;

	DWORD curTime = get_tick_count();
	float timeDelta = (curTime - lastTime)*0.001f;
	
	timeElapsed += timeDelta;

	frameCount++;
	//timeDelta is SPF (sec per frame)
	//printf("timeDelta : %f\n", timeDelta);
	//printf("cur : %d, last : %d\n", curTime, lastTime);
	stringstream ss_fps;
		ss_fps << "!!!!" << fps_tmp;	
		string str_fps = ss_fps.str();
		putText(src_status, str_fps, Point(160,120), FONT_HERSHEY_PLAIN, 1, Scalar(128, 0, 255), 1, 8, false);
	
	if(timeElapsed >= 1.0f){
		float fps = (float)frameCount/timeElapsed;
		fps_tmp = fps;
		//printf("frameCount : %d, timeElapsed : %f, fps : %f\n", frameCount, timeElapsed, fps);
		
		frameCount = 0;
		timeElapsed = 0.0f;
		
		//lastTime = curTime;
	}else{
	}
	lastTime = curTime;
}

