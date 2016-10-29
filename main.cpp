#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>

#include <stdio.h>
#include <string.h>

using namespace std;
using namespace cv;


Mat src, src_gray,src_gray_tmp,img_scene,src_HSV, src_H,src_S,src_V;
Mat milk1;
Mat dst, detected_edges, ROI;

int fd;
int edgeThresh = 1, G_x_min=320, G_x_max=0, G_y_min=240, G_y_max=0;
int lowThreshold;
int lowBlackThres=0, lowWhiteThres=255, lowRedThres=0, highRedThres=0, lowGreenThres=0, highGreenThres=0;
int diffRedThres=0, diffGreenThres=0;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
int green_cnt[6][8]={0,}, red_cnt[6][8]={0,}, edge_distance[7]={10,60,110,160,210,260,310}, edge_L=0, edge_R=0;
int a=0, look_at=0;

FILE *fp;


char* window_name = "Edge Map";

/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */

void MyLine( Mat img, Point start, Point end )
{
  int thickness = 2;
  int lineType = 8;
  line( img,
        start,
        end,
        Scalar( 0, 0, 255 ),
        thickness,
        lineType );
}

void on_trackbar (int, void*)
{

	
}

void CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 3x3
  blur( src_gray, detected_edges, Size(3,3) );

  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  /// Using Canny's output as a mask, we display our result
  dst = Scalar::all(0);

  src.copyTo( dst, detected_edges);
 }


int main(int argc, char** argv){
	fp = fopen("./threshold.txt","r");
	fscanf(fp, "%d", &lowRedThres);
	fscanf(fp, "%d", &highRedThres);
	fscanf(fp, "%d", &diffRedThres);

	fscanf(fp, "%d", &lowGreenThres);
	fscanf(fp, "%d", &highGreenThres);
	fscanf(fp, "%d", &diffGreenThres);

	fscanf(fp, "%d", &lowBlackThres);
	fscanf(fp, "%d", &lowWhiteThres);
	fscanf(fp, "%d", &lowThreshold);


//////////serial open start//////////////////

if ((fd = serialOpen ("/dev/serial0", 4800)) < 0)
  {
    //fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    printf("z");
	return 1 ;
  }

  if (wiringPiSetup () == -1)
  {
    //fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
	printf("a");
    return 1 ;
  }

//////////serial open end//////////////////


    milk1 = imread("./picture.jpg");
    //imshow("compare", milk1);
    VideoCapture cap(-1);

    if (!cap.isOpened())
    {
        cout << "Cannot open camera" << endl;
        return -1;
    }
   cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
   cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);

   // Create a window
   namedWindow( window_name, CV_WINDOW_AUTOSIZE );
   namedWindow("Output",CV_WINDOW_AUTOSIZE);

   serialPutchar (fd, 62); // init for looking foward




    while (1)
    {
        bool bSuccess = cap.read(src);

        if (!bSuccess)
        {
        cout << "Cannot read a frame from camera" << endl;
        break;
        }

	cvtColor( src, src_gray, CV_BGR2GRAY );
	cvtColor( src, src_HSV, COLOR_BGR2HSV );


        

	// Create a matrix of the same type and size as src (for dst)
	dst.create( src.size(), src.type() );
	src_gray_tmp.create( src_gray.size(), src_gray.type() );
	createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );


  

	// Create a Trackbar for user to enter threshold
	//  createTrackbar( "Max Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );
	// Show the image
	CannyThreshold(0, 0);


	int cnt[7]={0,};
	for(int i=0;i<6;i++){
		for(int j=0;j<8;j++){
			green_cnt[i][j]=0;
			red_cnt[i][j]=0;
		}
	}
	
	for(int i=0;i<7;i++){
		for(int j=200-1;j>=0;j--){
			if((int)dst.at<Vec3b>(j, edge_distance[i]).val[0]<30 && (int)dst.at<Vec3b>(j, edge_distance[i]).val[1]<30 && (int)dst.at<Vec3b>(7, edge_distance[i]).val[2]<30) cnt[i]++;
			else break;
		}

	}

	for(int i=0;i<7;i++){
		MyLine(dst, Point(edge_distance[i], 199-cnt[i]), Point(edge_distance[i], 199));
	}
	edge_L = (cnt[0]+cnt[1]+cnt[2])/3;
	edge_R = (cnt[3]+cnt[4]+cnt[5])/3;

	stringstream ss_L;
	ss_L << edge_L;
	string str_L = ss_L.str();
	putText(dst, str_L, Point2f(130, 100), FONT_HERSHEY_PLAIN, 0.7, Scalar(255, 255, 255), 1, 8, false);

	stringstream ss_R;
	ss_R << edge_R;
	string str_R = ss_R.str();

	putText(dst, str_R, Point2f(185, 100), FONT_HERSHEY_PLAIN, 0.7, Scalar(255, 255, 255), 1, 8, false);
	imshow( window_name, dst );
	imshow("Output", src);
	
	
	on_trackbar(0,0);
	int tmpt;
	tmpt = lowBlackThres;
	G_x_min=320, G_x_max=0, G_y_min=240, G_y_max=0;

	for(int i=0;i<240;i++){
		for(int j=0;j<320;j++){
			if(src_HSV.at<Vec3b>(i, j).val[0]<lowRedThres || src_HSV.at<Vec3b>(i, j).val[0]>highRedThres){
				int BG_tmp;
				BG_tmp = (src.at<Vec3b>(i, j).val[0]+src.at<Vec3b>(i, j).val[1])/2;
				if((src.at<Vec3b>(i, j).val[2]-BG_tmp)>diffRedThres){
					src.at<Vec3b>(i, j).val[0]=0;
					src.at<Vec3b>(i, j).val[1]=0;
					src.at<Vec3b>(i, j).val[2]=255;
					red_cnt[i/40][j/40]++;
				} 
				else{
					src.at<Vec3b>(i, j).val[0]=0;
					src.at<Vec3b>(i, j).val[1]=0;
					src.at<Vec3b>(i, j).val[2]=0;

				}
			}
			else if(lowGreenThres < src_HSV.at<Vec3b>(i, j).val[0] &&  src_HSV.at<Vec3b>(i, j).val[0]<highGreenThres){
				int RB_tmp;
				RB_tmp = (src.at<Vec3b>(i, j).val[0]+src.at<Vec3b>(i, j).val[2])/2;
				if((src.at<Vec3b>(i, j).val[1]-RB_tmp)>diffGreenThres){
					src.at<Vec3b>(i, j).val[0]=0;
					src.at<Vec3b>(i, j).val[1]=255;
					src.at<Vec3b>(i, j).val[2]=0;
					green_cnt[i/40][j/40]++;
					if(G_x_min>j) G_x_min = j;
					if(G_x_max<j) G_x_max = j;
					if(G_y_min>i) G_y_min = i;
					if(G_y_max<i) G_y_max = i;
				}
				else{
					src.at<Vec3b>(i, j).val[0]=0;
					src.at<Vec3b>(i, j).val[1]=0;
					src.at<Vec3b>(i, j).val[2]=0;

				}

			}
			else{
				src.at<Vec3b>(i, j).val[0]=0;
				src.at<Vec3b>(i, j).val[1]=0;
				src.at<Vec3b>(i, j).val[2]=0;
			}
		
			if(src_gray.at<unsigned char>(i,j)<lowBlackThres){
				src_gray.at<unsigned char>(i,j) = 0;
			}
			else if(src_gray.at<unsigned char>(i,j)>lowWhiteThres){
				src_gray.at<unsigned char>(i,j) = 255;
			}
			else{
				src_gray.at<unsigned char>(i,j) = 128;
			}


		}
	}

	//ROI = fullImage(Rect(G_x_min, G_y_min, G_x_max-G_x_min, G_y_max-G_y_min));
	if(G_x_max-G_x_min>0 && G_y_max-G_y_min>0){
		src_gray(Rect(G_x_min, G_y_min, G_x_max-G_x_min, G_y_max-G_y_min)).copyTo(ROI);
		imshow("ROI", ROI);
		//cout << "a";
	}
	for(int i=40; i<320;i+=40){
		line( src, Point2f( i, 0), Point2f( i, 239), Scalar(255, 0, 0), 1 );	
	}
	for(int j=40; j<240;j+=40){
		line( src, Point2f( 0, j), Point2f( 319, j), Scalar(255, 0, 0), 1 );	
	}	
	
	int green_max=0, green_max_index[2]={0,};
	int red_max=0, red_max_index[2]={0,};
	for(int i=0;i<6;i++){
		for(int j=0;j<8;j++){
			if(i<6-1 && j<8-1 && green_max<(green_cnt[i][j]+green_cnt[i][j+1]+green_cnt[i+1][j]+green_cnt[i+1][j+1])){
				green_max_index[0] = i;
				green_max_index[1] = j;
				green_max = green_cnt[i][j]+green_cnt[i][j+1]+green_cnt[i+1][j]+green_cnt[i+1][j+1];
			}
			if(i<6-1 && j<8-1 && red_max<(red_cnt[i][j]+red_cnt[i][j+1]+red_cnt[i+1][j]+red_cnt[i+1][j+1])){
				red_max_index[0] = i;
				red_max_index[1] = j;
				red_max = red_cnt[i][j]+red_cnt[i][j+1]+red_cnt[i+1][j]+red_cnt[i+1][j+1];
			}
			stringstream ss;
			ss << green_cnt[i][j];
			string str = ss.str();
			putText(src, str, Point2f(j*40, (i+1)*40), FONT_HERSHEY_PLAIN, 0.7, Scalar(255, 255, 255), 1, 8, false);
		}
	}
	rectangle(src, Point(green_max_index[1]*40, green_max_index[0]*40), Point(green_max_index[1]*40+80, green_max_index[0]*40+80), Scalar(255, 255, 255), 0, 8);
	if(red_max>1700){
		rectangle(src, Point(red_max_index[1]*40, red_max_index[0]*40), Point(red_max_index[1]*40+80, red_max_index[0]*40+80), Scalar(0, 255, 255), 0, 8);
		//cout << red_max_index[0] << " " << red_max_index[1] << endl;
		if(red_max_index[1] == 3 && a==0){
			serialPutchar (fd, 84);
			a=-1;
			cout << "go to the ball" << endl;
		}
	}
	cout << "!" << a << endl;
	//cout << "i: " << green_max_index[0] << " j: " << green_max_index[1] << " max: " << green_max << endl;
	if(green_max_index[1]<3 && green_max>100 && a==0){
		serialPutchar (fd, 49); // 
		a=-1;
		cout << "move left" << endl;
	}
	if(green_max_index[0]>3 && green_max>100 && a==0 && look_at==0){
		serialPutchar (fd, 62); // 
		a=-1;
		look_at = -1;
		cout << "look at down" << endl;
	}

	if(green_max_index[1]>4 && green_max>100 && a==0){
		serialPutchar (fd, 42); // 
		a=-1;
		cout << "move right" << endl;
	}

	if(look_at!=0 && green_max < 100 && a==0){
		serialPutchar (fd, 61); // 
		a=-1;
		look_at = 0;
		cout << "look at foward" << endl;
	}
	if(look_at==-1 && green_max > 100 && a==0 && (green_max_index[1]==3 || green_max_index[1]==4) && green_max_index[0]<2){
		serialPutchar (fd, 65);
		a=-1;
		cout << "move foward" << endl;
	}
	if(look_at==0 && green_max > 100 && a==0 && green_max_index[0]<=3){
		serialPutchar (fd, 65);
		a=-1;
		cout << "move foward" << endl;
	}
	if(look_at==-1 && green_max > 100 && a==0 && (green_max_index[1]==3 || green_max_index[1]==4) && (green_max_index[0]>=2)){
		serialPutchar (fd, 71); // 
		a=-1;
		cout << "hold milk" << endl;
	}
	if(edge_L-edge_R>20){
		serialPutchar (fd, 37); // 
		a=-1;
		cout << "Right" << endl;
	}
	if(edge_R-edge_L>20){
		serialPutchar (fd, 44); // 
		a=-1;
		cout << "Left" << endl;

	}
	imshow("Hue", src);
	createTrackbar( "Red Min:", "Hue", &lowRedThres, 255, on_trackbar );
	createTrackbar( "Red Max:", "Hue", &highRedThres, 255, on_trackbar );
	createTrackbar( "Red Diff:", "Hue", &diffRedThres, 255, on_trackbar );
	createTrackbar( "Green Min:", "Hue", &lowGreenThres, 255, on_trackbar );
	createTrackbar( "Green Max:", "Hue", &highGreenThres, 255, on_trackbar );
	createTrackbar( "Green Diff:", "Hue", &diffGreenThres, 255, on_trackbar );
	//cout << "T" << lowBlackThres << endl;
	
	threshold(src, src, 0, lowBlackThres, CV_THRESH_BINARY);
	imshow("Grayscale", src_gray);
	createTrackbar( "Black Threshold:", "Grayscale", &lowBlackThres, 255, on_trackbar );
	createTrackbar( "White Threshold:", "Grayscale", &lowWhiteThres, 255, on_trackbar );

	//serialPutchar (fd, (unsigned char)i);



    while (serialDataAvail (fd) && a!=0)
    {
	//printf("ready\n");
	printf("!%d\n",fd);
	a=serialGetchar (fd);
        printf (" -> %3d\n", a) ;
	a = -2;
        fflush (stdout) ;
    }
    if(a==-2){
	a = 0;
	delay(100);
    }
	//cout << a << endl;

//	fflush (stdout) ;
	if (waitKey(30) == 27)
        {
		printf("key\n");
        	cout << "Exit" << endl;
		//serialPutchar (fd, (unsigned char)7);
	        break;
        }

    }

    fclose(fp);
	fp = fopen("./threshold.txt", "w");
	fprintf(fp, "%d\n", lowRedThres);
	fprintf(fp, "%d\n", highRedThres);
	fprintf(fp, "%d\n", diffRedThres);

	fprintf(fp, "%d\n", lowGreenThres);
	fprintf(fp, "%d\n", highGreenThres);
	fprintf(fp, "%d\n", diffGreenThres);

	fprintf(fp, "%d\n", lowBlackThres);
	fprintf(fp, "%d\n", lowWhiteThres);
	fprintf(fp, "%d\n", lowThreshold);

    fclose(fp);
    return 0;
}
