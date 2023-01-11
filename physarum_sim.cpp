#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <omp.h>

//using namespace std;
//using namespace cv;

class Agent
{
public:
	std::vector<float> position = {5.0,5.0};
	float angle = 0.0;
	int mask;
};

//max proxy
float cmp(float num, float num2)
{
if(num > num2){
	return num;
}
	return num2;
}

//min proxy
float cmp2(float num, float num2)
{
if(num < num2){
        return num;
}
        return num2;
}

//function to emulate de evaporation of the trails
void evaporate(Eigen::MatrixXd & imageB, Eigen::MatrixXd & imageG, float factor = 0.02){
	#pragma omp parallel for
	for(int i = 0; i < imageB.rows(); i++){
		for(int j = 0; j < imageB.cols(); j++){
			imageB(i,j)=imageB(i,j)*factor;
			imageG(i,j)=imageG(i,j)*factor;
		}
	}
}
// emulate de spreading of the trails
void difusion(Eigen::MatrixXd & imageB,
			   Eigen::MatrixXd & imageG,
			   float width, float height){
	int nr = height-1;
	int nc = width-1;
	Eigen::MatrixXd baseB(imageB.rows(),imageB.cols());
	Eigen::MatrixXd baseG(imageG.rows(),imageG.cols());
	#pragma omp parallel for
	for(int i = 1; i < nr; i++){
		for(int j = 1; j < nc; j++){
			baseB(i,j)=imageB(i-1,j+1)+imageB(i,j+1)+imageB(i+1,j+1) +
						imageB(i-1,j)+imageB(i,j)+imageB(i+1,j) +
						imageB(i-1,j-1)+imageB(i,j-1) +imageB(i+1,j-1);

			baseG(i,j)=imageG(i-1,j+1)+imageG(i,j+1)+imageG(i+1,j+1) +
						imageG(i-1,j)+imageG(i,j)+imageG(i+1,j) +
						imageG(i-1,j-1)+imageG(i,j-1) +imageG(i+1,j-1);
		}
	}

	baseB(0,0) = imageB(0,0)+imageB(1,0)+imageB(0,0)+imageB(1,1);
	baseB(nr,0) = imageB(nr,0)+imageB(nr,1)+imageB(nr-1,0)+imageB(nr-1,1);
	baseB(0,nc) = imageB(0,nc)+imageB(1,nc)+imageB(0,nc-1)+imageB(1,nc-1);
	baseB(nr,nc) = imageB(nr,nc)+imageB(nr-1,nc)+imageB(nr,nc-1)+imageB(nr-1,nc-1);

	baseG(0,0)=imageG(0,0)+imageG(1,0)+imageG(0,0)+imageG(1,1);
	baseG(nr,0) = imageG(nr,0)+imageG(nr,1)+imageG(nr-1,0)+imageG(nr-1,1);
	baseG(0,nc) = imageG(0,nc)+imageG(1,nc)+imageG(0,nc-1)+imageG(1,nc-1);
	baseG(nr,nc) = imageG(nr,nc)+imageG(nr-1,nc)+imageG(nr,nc-1)+imageG(nr-1,nc-1);

	for(int i = 1; i<nc;i++){
		//upper row
		baseB(nr,i) = imageB(nr,i-1)+imageB(nr,i)+imageB(nr,i+1)+
				imageB(nr-1,i-1)+imageB(nr-1,i)+imageB(nr-1,i+1);
		//bottom row
		baseB(0,i) = imageB(0,i-1) + imageB(0,i) + imageB(0,i+1)+
				imageB(1,i-1) + imageB(1,i) + imageB(1,i+1);
		//upper row
		baseG(nr,i)=imageG(nr,i-1)+imageG(nr,i)+imageG(nr,i+1)+
				imageG(nr-1,i-1)+imageG(nr-1,i)+imageG(nr-1,i+1);
		//bottom row
		baseG(0,i) = imageG(0,i-1)+imageG(0,i)+imageG(0,i+1)+
				imageG(1,i-1)+imageG(1,i)+imageG(1,i+1);
	}
	for(int i = 1;i<nr;i++){
		//left col
		baseB(i,nc)=imageB(i-1,nc)+imageB(i,nc)+imageB(i+1,nc)+
			imageB(i-1,nc-1)+imageB(i,nc-1)+imageB(i+1,nc-1);
		//right col
		baseB(i,0)=imageB(i-1,0)+imageB(i,0)+imageB(i+1,0)+
			imageB(i-1,1)+imageB(i,1)+imageB(i+1,1);
		//left col
		baseG(i,nc)=imageG(i-1,nc)+imageG(i,nc)+imageG(i+1,nc)+
			imageG(i-1,nc-1)+imageG(i,nc-1)+imageG(i+1,nc-1);
		//right col
		baseG(i,0)=imageG(i-1,0)+imageG(i,0)+imageG(i+1,0)+
			imageG(i-1,1)+imageG(i,1)+imageG(i+1,1);
	}
	imageB = baseB/9;
	imageG = baseG/9;
}

std::vector<float> sense(Agent & agent,Eigen::MatrixXd & imageB,
							Eigen::MatrixXd & imageG,
							float offset,int width,
							int height,float dist = 2.0){
    float sensorAngle = agent.angle + offset;
    std::vector<float> sensorCenter = {agent.position[0] + sin(sensorAngle) * dist, agent.position[1] + cos(sensorAngle) * dist};
    if(sensorCenter[0] < 0 || sensorCenter[1]<0 || sensorCenter[0] >= height || sensorCenter[1] >= width){
        std::vector<float> out = {0.0,0.0};
        return out;
    }else{
        std::vector<float> out = {(float)imageB((int)sensorCenter[0],(int)sensorCenter[1]),
        						  (float)imageG((int)sensorCenter[0],(int)sensorCenter[1])};
    	return out;
    }
}

//Function to update the agents positions in each step
void update(Agent & agent,Eigen::MatrixXd & imageB,
			Eigen::MatrixXd & imageG,
			float offset,float dist, float width,
			float height,float threshold=0.6,
			float speed = 3.0)
{
	std::vector<float> newpos = {0.0,0.0};

	float offset1 = M_PI/offset;
	float angle = agent.angle;

    std::vector<float> forward_v = sense(agent,imageB,imageG,0.0,width,height,dist);
    std::vector<float> right_v = sense(agent,imageB,imageG,offset1,width,height,dist);
    std::vector<float> left_v = sense(agent,imageB,imageG,-offset1,width,height,dist);

    float forward;
    float right;
    float left;

	// evaluate the direction with the higher diference (same species trail - other species trail).
	// and a random angle in that direction
    if(agent.mask ==1){
        forward = forward_v[0]-forward_v[1];
        right = right_v[0]-right_v[1];
        left = left_v[0]-left_v[1];

        if(forward<threshold){
            if(forward>right){
	            if(forward > left){
			        angle = angle;
			        }
		        }
		    }

        if(right > left){
        	if(right < threshold){
	       	angle=angle+(rand_r()) / static_cast <float> (RAND_MAX) *offset1;
        	}
        }
        if(right < left){
        	if(left < threshold){
        	angle=angle-(rand_r()) / static_cast <float> (RAND_MAX) *offset1;
        	}
        }
    }
    if(agent.mask == -1){
        forward = forward_v[1]-forward_v[0];
        right = right_v[1]-right_v[0];
        left = left_v[1]-left_v[0];

        if(forward<threshold){
            if(forward>right){
	            if(forward > left){
			        angle = angle;
			        }
		        }
		    }
        if(right > left){
        	if(right < threshold){
	       	angle=angle+(rand_r()) / static_cast <float> (RAND_MAX) *offset1;
        	}
        }
        if(right < left){
        	if(left < threshold){
        	angle=angle-(rand_r()) / static_cast <float> (RAND_MAX) *offset1;
        	}
        }
    }

	//if going forward hits another particle, change the direction
	if(forward >= threshold){
		angle = angle + M_PI  -M_PI/4 +(rand_r()) / static_cast <float> (RAND_MAX) *M_PI/2;
	}

	agent.angle = angle;
	newpos[0]=agent.position[0]+sin(agent.angle)*speed;
    newpos[1]=agent.position[1]+cos(agent.angle)*speed;

	//if an agent hit the map boundaries get a new random angle
	if(newpos[0] < 0 || newpos[1] < 0 || newpos[0] >= height || newpos[1] >= width){
		newpos[0] = cmp2(height-0.01,cmp(0.0,newpos[0]));
		newpos[1] = cmp2(width-0.01,cmp(0.0,newpos[1]));
		agent.angle = (rand_r()) / static_cast <float> (RAND_MAX) * 2 * M_PI;
	};
	//update agent position
	agent.position[0]=newpos[0];
	agent.position[1]=newpos[1];
}

int main(){
	float WIDTH;
	float HEIGHT;
	typedef cv::Vec<float,1> Vec1f;
	std::cout<<"Width: "<<std::endl;
	std::cin>>WIDTH;
	std::cout<<"Height:"<<std::endl;
	std::cin>>HEIGHT;

	//initialize trail maps
	Eigen::MatrixXd imageB((int)HEIGHT,(int)WIDTH);
	Eigen::MatrixXd imageG((int)HEIGHT,(int)WIDTH);
	Eigen::MatrixXd imageR((int)HEIGHT,(int)WIDTH);

	int n_threads;
	std::cout<<"N Threads:"<<std::endl;
	std::cin>>n_threads;
    omp_set_num_threads(n_threads);

	//create opencv mat for the visualitation
	cv::Mat fin((int)HEIGHT,(int)WIDTH, CV_64FC3, cv::Scalar(1,1,1,1));
	cv::Mat blue(cv::Size((int)HEIGHT,(int)WIDTH), CV_64FC1, cv::Scalar(0));
	cv::Mat green(cv::Size((int)HEIGHT,(int)WIDTH), CV_64FC1, cv::Scalar(0));
	cv::Mat red(cv::Size((int)HEIGHT,(int)WIDTH), CV_64FC1, cv::Scalar(0));

	//Initialize trail maps to cero
	cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
	for(int i = 0; i<HEIGHT;i++){
	        for(int j=0;j<WIDTH;j++){
	            imageB(i,j)=0;
	            imageG(i,j)=0;
	            imageR(i,j)=0;
        	}
	}
	cv::eigen2cv(imageR,red);

	// Initialize agents at random position inside a circle
	// looking at random directions
	int n_agents;
	std::cout<<"Number of particles:"<<std::endl;
	std::cin>>n_agents;
	std::vector<Agent> agents;

	for(int i = 0; i<n_agents;i++){
		Agent a1;
		float theta = (rand_r()) /static_cast <float> (RAND_MAX)*2*M_PI;
		float r = (rand_r()) /static_cast <float> (RAND_MAX)* (HEIGHT/2-5);
		float sign = -1 + (rand_r()) / static_cast <float> (RAND_MAX);
		a1.position[1] =  WIDTH/2 + r * cos(theta);
		a1.position[0] =  HEIGHT/2 + r * sin(theta);
		a1.angle = (rand_r()) /static_cast <float> (RAND_MAX)*2*M_PI;
		float mask = 0.5 - (rand_r()) / static_cast <float> (RAND_MAX);
		a1.mask = mask/abs(mask);
		agents.push_back(a1);
	}



	// update agent positions in a loop
	float speed;
	float threshold;
	float factor;
	float offset;
	float dist;
	std::cout<<"Speed:"<<std::endl;
	std::cin>>speed;
	std::cout<<"Threshold:"<<std::endl;
	std::cin>>threshold;
	std::cout<<"Evaporation rate:"<<std::endl;
	std::cin>>factor;
	std::cout<<"Sensor distance:"<<std::endl;
	std::cin>>dist;
	std::cout<<"Sensor angle:"<<std::endl;
	std::cin>>offset;
	n_agents = agents.size();
	for(int i =0; i<100000; i++){
		evaporate(imageB,imageG,factor);
	    difusion(imageB,imageG,imageB.cols(),imageB.rows());
		#pragma omp parallel for
		for(int j =0; j<n_agents; j++){
			update(agents[j],imageB,imageG,offset,dist,WIDTH,HEIGHT,threshold,speed);
			int ind1= (int)agents[j].position[0];
			int ind2= (int)agents[j].position[1];
			if(agents[j].mask>0){
				imageB(ind1,ind2)=1;
				}
			if(agents[j].mask<0){
				imageG(ind1,ind2)=1;
			}
		}

		cv::eigen2cv(imageB,blue);
		cv::eigen2cv(imageG,green);

		#pragma omp parallel for
		for(int p = 0; p<fin.cols*2;p++){
			for(int k=0;k<fin.rows;k++){
			    fin.at<cv::Vec3f>(k,p).val[0]=blue.at<float>(cv::Point(p,k))*1.0;
			    fin.at<cv::Vec3f>(k,p).val[2]=blue.at<float>(cv::Point(p,k))*0.5;
			    fin.at<cv::Vec3f>(k,p).val[4]=blue.at<float>(cv::Point(p,k))*0.05;
			    fin.at<cv::Vec3f>(k,p).val[0]=cmp2(255,
			            fin.at<cv::Vec3f>(k,p).val[0] +
			            green.at<float>(cv::Point(p,k))*0.05);
			    fin.at<cv::Vec3f>(k,p).val[2]=cmp2(255,
			            fin.at<cv::Vec3f>(k,p).val[2] +
			            green.at<float>(cv::Point(p,k))*0.5);
			    fin.at<cv::Vec3f>(k,p).val[4]=cmp2(255,
			            fin.at<cv::Vec3f>(k,p).val[4] +
			            green.at<float>(cv::Point(p,k))*1);
		    	}
		}
	    cv::imshow("Display Image", fin);


//  Saving snapshots
	    if(i==25){
	    	cv::Mat3b image1;
	    	fin.convertTo(image1,CV_8UC3,255);
	        imwrite("../i1.jpeg",  image1);
	    }
	    if(i==60){
	    	cv::Mat3b image2;
	    	fin.convertTo(image2,CV_8UC3,255);
	        imwrite("../i2.jpeg",  image2);
	    }
	    if(i==120){
	    	cv::Mat3b image3;
	    	fin.convertTo(image3,CV_8UC3,255);
	        imwrite("../i3.jpeg",  image3);
	    }
	    if(i==160){
	    	cv::Mat3b image4;
	    	fin.convertTo(image4,CV_8UC3,255);
	        imwrite("../i4.jpeg",  image4);
	    }
	    cv::waitKey(1);
	}


	return 0;
}
