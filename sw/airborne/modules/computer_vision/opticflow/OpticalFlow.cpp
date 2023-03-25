//
// Created by SerbiBlaga on 24/03/2023.
//

//struct flow_t{
// struct point_t pos;
// int32_t        flow_x;
// int32_t        flow_y;
// uint32_t       error;   -- matching error in the tracking process in the subpixels
// }



#include <stdio.h>
#include "image.h"
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/video/tracking.hpp>

using namespace cv;
using namespace std;


int determine_flow(char *prev, char *curr, int height, int width){

    //struct opticflow* flow = new struct opticflow;

    //Copy and scale the images to M1 and M2
    Mat M1(height, width, CV_8UC2, prev);
    Mat M2(height, width, CV_8UC2, curr);

    Mat prev_bgr;
    Mat bgr;

    int crop_height = 30;
    int crop_width = 10;

    // Crop image
    Rect crop_image;
    crop_image.x = crop_height;
    crop_image.y = crop_width;
    crop_image.width = width - 2 * crop_image.x; //crop the image by removing twice the x-direction corners
    crop_image.height = height - 2 * crop_image.y; //crop the image by removing twice the y-direction corners

    //Convert to gray
    cvtColor(M1(crop_image), prev_bgr, CV_YUV2GRAY_Y422);
    cvtColor(M2(crop_image), bgr, CV_YUV2GRAY_Y422);

    Mat flow(prev_bgr.rows, prev_bgr.cols, CV_32FC2); //matrix to store the flows

    Mat img_blur;
    blur(prev_bgr, img_blur, Size(10, 10)); //blur blur blur cause we can

    Mat thresh;
    threshold(img_blur, thresh, 100, 255, THRESH_BINARY); //threshold calculation

    //now it's time to find and draw the contour
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(thresh, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
    int max_area = -1;
    int max_contour_index = -1;

    for (int i = 0; i < contours.size(); i++){
        double area = contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_contour_index = i;
        }
    }

    vector<Point> largest_contour = contours[max_contour_index];
    Mat mask = Mat::zeros(prev_bgr.size(), CV_8UC1);
    drawContours(mask, vector<vector<Point>>{largest_contour}, 0, Scalar(255), -1);

    // Concatenate all the points in the contours
    vector<Point2f> points_old;
    for (size_t i = 0; i < contours.size(); i++) {
        for (size_t j = 0; j < contours[i].size(); j++) {
            points_old.push_back(contours[i][j]);
        }
    }

    // Parameters for lucas kanade optical flow
    Size winSize(15, 15);
    int maxLevel = 2;
    TermCriteria criteria(TermCriteria::COUNT | TermCriteria::EPS, 10, 0.03);

    // calculate optical flow
    vector<Point2f> points_new;
    vector<uchar> status;
    vector<float> error;
    calcOpticalFlowPyrLK(prev_bgr, bgr, points_old, points_new, status, error, winSize, maxLevel, criteria);

    // filter the flow vector by their status
    vector<Point2f> good_points_old, good_points_new, flow_vectors;
    for (size_t i = 0; i < status.size(); i++) {
        if (status[i]) {
            good_points_old.push_back(points_old[i]);
            good_points_new.push_back(points_new[i]);
            flow_vectors.push_back(points_new[i] - points_old[i]);
        }
    }

    // filter the flow vectors by their magnitude
    vector<float> magnitudes;
    for (size_t i = 0; i < flow_vectors.size(); i++) {
        magnitudes.push_back(sqrt(pow(flow_vectors[i].x, 2) + pow(flow_vectors[i].y, 2)));
    }

    // find the flow vectors that are above the threshold and save them
    float threshold = 0.9 * (*max_element(magnitudes.begin(), magnitudes.end()));
    vector<Point2f> good_flow_vectors;
    for (size_t i = 0; i < flow_vectors.size(); i++) {
        if (magnitudes[i] >= threshold) {
            good_flow_vectors.push_back(flow_vectors[i]);
            good_points_old.push_back(good_points_old[i]);
            good_points_new.push_back(good_points_new[i]);
        }
    }
    
    return *flow_t;


}