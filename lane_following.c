/* This is a program that uses OpenCV and Serial Communication to enable a robot to follow a black lane between
   white lines as borders. It was the final product of the seven day Winter Workshop organized for first year
   students by Robotix, IIT Kharagpur. The participants began with no prior knowledge of Computer Vision
   and ended up coding the robot to perform this task 

   The program used a custom implementation of the Hough Line Transform to detect the lanes and follow it accordingly*/ 

#include "stdafx.h"
#include <stdio.h>
#include <highgui.h>
#include <cv.h>
#include <cxcore.h>
#include <stdlib.h>
#include <math.h>
#include "tserial.h"
#include "bot_control.h"

serial comm;

#define IMGDATA(image,i,j,k) (((uchar *)image->imageData)[(i)*(image->widthStep) + (j)*(image->nChannels) + (k)])

/* This function will calculate the parameters of that line, and increases the value of correspondending array element in matrix by 1 */
void draw_graph(int **matrix, int y,int x, int r_max, int t_max)
{
  int r,t;
  for(t=0;t<t_max;t++)
    {
      r = x*cos((float)t/100) + y*sin((float)t/100);
      if(r>=0)
	matrix[r][t]++;
    }
}


void show_line(IplImage *img, int r0, int t0)
{
  int x,y,ht,wd;
  ht = img->height;
  wd = img->width;
  //printf(" %d %d \n",r0,t0);

  for(x=0;x<wd;x++)
    {
      y= (float)r0/sin((float)t0/100) - x/tan((float)t0/100);
      if(y>=0 && y<ht)
	{
                        
	  IMGDATA(img,y,x,0)=0;
	  IMGDATA(img,y,x,1)=0;
	  IMGDATA(img,y,x,2)=255;
	}
    }
  for(y=0;y<ht;y++)
    {
      x= (float)r0/cos((float)t0/100) - y*tan((float)t0/100);
      if(x>=0 && x<wd)
	{
                        
	  IMGDATA(img,y,x,0)=0;
	  IMGDATA(img,y,x,1)=0;
	  IMGDATA(img,y,x,2)=255;
	}
    }

}


void line_detection(IplImage *line,IplImage *img,int th, CvPoint *p,int *count)		//Using Hough Transform
{
  int i,j,ht,wd;
  *count=0;
  ht = img->height;
  wd = img->width;
  int r_max = sqrt((float)(ht*ht+wd*wd))+1;		//Maximum value of parameter r possible
  /*The two dimensions of the accumulator array would correspond to quantized values for (r,?).
    Here multiplication factor of 100 is taken to quantized value of ?
    Hence maximum value of quantized ? is 100*2*3.14 ? 630 */
  int **matrix;									
  /*Dyanamic memory allocation for 2D array matrix  */
  matrix=(int **)malloc(r_max*sizeof(int *));		
  for(i=0;i<r_max;i++)
    matrix[i]=(int *)malloc(630*sizeof(int));	
  int r,t;										//Variables for quantized values for (r,?)
  /* Initialising the each element of array to zero */
  for(r=0;r<r_max;r++)
    {
      for(t=0;t<630;t++)
	matrix[r][t]=0;
    }
  for(i=0;i<ht;i++)
    {
      for(j=0;j<wd;j++)
	{
	  if(IMGDATA(img,i,j,0)>250)					//If the white pixel is detected (i.e., edge of a white lane) then we apply Hough Transform to that pixel
	    draw_graph(matrix,i,j ,r_max,630);
	}
    }

  /*Code for detecting the local maxima (count) in array matrix which is achieved by traversing matrix kernel by kernel of dimension of "rblock by tblock".
    If this maxima is greater than the threshold set in variable th, then corresponding  quantized values for (r,?) is stored is an array p 
    and a line is drawn in red color in an IplImage *line */
  int r_m=0,t_m=0,r_min,t_min;
  int rblock=20;
  int tblock=50;
  int max=0;
  int rr=0,tt=0;
  for(r=0;r<r_max;r+=rblock)
    {
      for(t=0;t<630;t+=tblock)
	{
	  r_m=r+rblock<r_max?r+rblock:r_max-1;	//to avoid the value to r outside is range
	  t_m=t+tblock<630?t+tblock:630-1;
	  for(i=r;i<r_m;i++)
	    {
	      for(j=t;j<t_m;j++)
		{
		  if(matrix[i][j]>max)
		    {
		      max=matrix[i][j];
		      rr=i;
		      tt=j;
		    }
		}
	    }
	  if(matrix[rr][tt]>th)
	    {
	      show_line(line,i,j);
	      p[*count].x=i;				//p[*count].x storing quantized value for r
	      p[*count].y=j;				//p[*count].y storing quantized value for ?
	      (*count)++;
	    }
	  max=0;
	}
    }
  for(i=0;i<r_max;i++)
    free(matrix[i]);
}



void main(){
  comm.startDevice("COM8", 9600);
  IplImage* image; IplImage* gscale,*line;
  CvCapture* capture = cvCreateCameraCapture(1);	//The video is loaded into a pointer of type CvCapture
  int th=145;
  char *win="modified";
  cvNamedWindow("original", CV_WINDOW_AUTOSIZE);//Window created
  cvNamedWindow(win,CV_WINDOW_AUTOSIZE); //To display the frames
  image = cvQueryFrame(capture);
  CvPoint *p;
  p=(CvPoint*)malloc(10000*sizeof(CvPoint));
  int x,y;
  int i,j,th1=10,th2=10,thr=300,thl=300;
  int count=0;
  int y_mid=(image->height)/2;

  cvCreateTrackbar("Hough transform curve Threshold",win,&th,1000,NULL);
  cvCreateTrackbar("Dilution threshold",win,&th1,50,NULL);
  cvCreateTrackbar("Errosion threshold ",win,&th2,50,NULL);
  cvCreateTrackbar("Right edge detection threshold",win,&thr,image->width,NULL);
  cvCreateTrackbar("Left edge detection threshold",win,&thl,image->width,NULL);


                 
  while(1)
    {
      image = cvQueryFrame(capture);
      line = cvCreateImage(cvSize(image->width,image->height),IPL_DEPTH_8U,3);
      gscale = cvCreateImage(cvSize(image->width,image->height),IPL_DEPTH_8U,1);
      cvShowImage("original",image);
      cvCvtColor(image,gscale,CV_BGR2GRAY);
      cvDilate(gscale,gscale,NULL,th1);
      cvErode(gscale,gscale,NULL,th2);
      // cvShowImage("original1",gscale);
      cvCanny(gscale,gscale,100,100,3);
      count=0;
      line_detection(line,gscale,th,p,&count);  

      int x_r=0,x_l=0,c_r=0,c_l=0;
      float t_r=0.0,t_l=0.0;
      for(i=0;i<count;i++)
        {
	  x= (float)p[i].x/cos((float)p[i].y/100) - y_mid*tan((float)p[i].y/100);
	  if(x <= (image->width)/2)			//If point of line at y=y_mid is in left side of  image
	    {
	      x_l+=x;						//Summing their x-coordinates
	      c_l++;						//Counting number of such lines
	      t_l+=(float)p[i].y/100;		//Summing their angle (?) in (r,?) form
	    }
	  else								//If point of line at y=y_mid is in right side of  image
	    {
	      x_r+=x;						//Summing their x-coordinates
	      c_r++;						//Counting number of such lines
	      t_r+=(float)p[i].y/100;		//Summing their angle (?) in (r,?) form
	    }
        }
      if(c_r==0)
	c_r=1;
      if(c_l==0)
	c_l=1;
      t_l=t_l/c_l;		//t_l now stores average value of ? in (r,?) form of the lines in left side of  image
      t_r=t_r/c_r;		//t_r now stores average value of ? in (r,?) form of the lines in right side of  image
      x_r=x_r/c_r;		//x_r now stores average value of x coordinate of the lines in right side of  image
      x_l=x_l/c_l;		//x_l now stores average value of y coordinate of the lines in left side of  image
   		

      //printf(" %f %f \n",t_l,t_r);
      int j_min=0,j_max=0;

        
      if(t_r==0.0)						//if no line is detected in right
        {
	  comm.send_data('d');
	  printf("Right\n");
        }
      else if(t_l==0.0)					//if no line is detected in left
        {
	  comm.send_data('a');
	  printf("Left\n");
        }
      else if(t_l!=0.0 && t_r!=0.0)
        {
	  // printf("%d\n",x);
	  if(x<thr && x>thl)			
	    {
	      comm.send_data('w');
	      printf("Straight\n");
	    }
	  else if(x>=thr)
	    {
	      comm.send_data('a');
	      printf("Left\n");
	    }

	  else if(x<=thl)
	    {
	      comm.send_data('d');
	      printf("Right\n");
	    }
        }
      else
        {
	  comm.send_data('d');
	  printf("Right\n");
        }
      cvShowImage(win,line);

      cvWaitKey(33);
    }
  cvReleaseImage(&image); cvReleaseImage(&gscale); cvReleaseImage(&line);
  cvDestroyWindow(win); //Window is closed
  comm.stopDevice();
}


