/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/cv_opencvdemo.c"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */

//
// Created by SerbiBlaga on 24/03/2023.
//

#include "OpticFlow.h"
#include <iostream>
#include <stdio.h>
#include "lib/vision/image.h"
#include <stdlib.h>
#include <cstdint>
#include "opencv_image_functions.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/video/tracking.hpp>
//#include "linear_flow_fit.h"
// #include "size_divergence.h"

#define PRINT(string, ...) fprintf(stderr, "[mav_exercise->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#include <math.h>
#include <string.h>
//#include "defs_and_types.h"
// #include "math/pprz_algebra_float.h"
// #include "math/pprz_matrix_decomp_float.h"
// #include "math/pprz_simple_matrix.h"

using namespace cv;
using namespace std;

// Is this still necessary?
#define MAX_COUNT_PT 50

#define MIN_SAMPLES_FIT 3

/**
 * Analyze a linear flow field, retrieving information such as divergence, surface roughness, focus of expansion, etc.
 * @param[out] outcome If 0, there were too few vectors for a fit. If 1, the fit was successful.
 * @param[in] vectors The optical flow vectors
 * @param[in] count The number of optical flow vectors
 * @param[in] error_threshold Error used to determine inliers / outliers.
 * @param[in] n_iterations Number of RANSAC iterations.
 * @param[in] n_samples Number of samples used for a single fit (min. 3).
 * @param[in] im_width Image width in pixels
 * @param[in] im_height Image height in pixels
 * @param[out] info Contains all info extracted from the linear flow fit.
 */



/**
 * Analyze a linear flow field, retrieving information such as divergence, surface roughness, focus of expansion, etc.
 * @param[in] vectors The optical flow vectors
 * @param[in] count The number of optical flow vectors
 * @param[in] error_threshold Error used to determine inliers / outliers.
 * @param[in] n_iterations Number of RANSAC iterations.
 * @param[in] n_samples Number of samples used for a single fit (min. 3).
 * @param[out] parameters_u* Parameters of the horizontal flow field
 * @param[out] parameters_v* Parameters of the vertical flow field
 * @param[out] fit_error* Total error of the finally selected fit
 * @param[out] min_error_u* Error fit horizontal flow field
 * @param[out] min_error_v* Error fit vertical flow field
 * @param[out] n_inliers_u* Number of inliers in the horizontal flow fit.
 * @param[out] n_inliers_v* Number of inliers in the vertical flow fit.
 */


struct flow_t *determine_flow(char *prev, char *curr, int height, int width, uint16_t winSize_i, uint16_t maxLevel, int *array_size){
    //struct flow_t* flow = new struct flow_t;
    // PRINT("STARTDETER");
    vector<flow_t> lin_vectors;
    //   PRINT("DETERMINE??");

    //Copy and scale the images to M1 and M2
    Mat M1(height, width, CV_8UC2, prev);
    Mat M2(height, width, CV_8UC2, curr);

    Mat prev_bgr;
    Mat bgr;

    int crop_height = 0;
    int crop_width = 0;

    // Crop image
    Rect crop_image;
    crop_image.y = crop_height;
    crop_image.x = crop_width;
    crop_image.width = width - 2 * crop_image.x; //crop the image by removing twice the x-direction corners
    crop_image.height = height - 2 * crop_image.y; //crop the image by removing twice the y-direction corners
    //width = crop_image.width;
    //height = crop_image.height;

    //colorbgr_opencv_to_yuv422(prev_bgr, prev);
    //colorbgr_opencv_to_yuv422(bgr, curr);

    //Convert to gray
    cvtColor(M1(crop_image), prev_bgr, CV_YUV2GRAY_Y422);
    cvtColor(M2(crop_image), bgr, CV_YUV2GRAY_Y422);

    // float scale = 0.2;
    // resize(prev_bgr, prev_bgr, Size(), scale, scale, CV_INTER_LINEAR);
    // resize(bgr, bgr, Size(), scale, scale, CV_INTER_LINEAR);


    //grayscale_opencv_to_yuv422(prev_bgr, prev);
    //grayscale_opencv_to_yuv422(bgr, curr);

    //Mat flow(prev_bgr.rows, prev_bgr.cols, CV_32FC2); //matrix to store the flows

    Mat img_blur;
    blur(prev_bgr, img_blur, Size(10, 10)); //blur blur blur cause we can

    Mat thresh;
    threshold(img_blur, thresh, 100, 255, THRESH_BINARY); //threshold calculation

    //now it's time to find and draw the contour
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(thresh, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
    if(contours.size() == 0){
        *array_size = 0;
        return nullptr;
        
    }
    int max_area = -1;
    int max_contour_index = -1;
    // PRINT("CONTOURS");
    PRINT("%d", contours.size());
    for (size_t i = 0; i < contours.size(); i++){
        double area = contourArea(contours[i]);
        // PRINT("PULA MARE");
        if (area > max_area) {
            max_area = area;
            max_contour_index = i;
        }
    }
    // PRINT("SUCKIT");
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
    //Size winSize(15, 15);

    uint16_t nr = 12;
    nr = winSize_i;

    Size winSize(nr, nr);

    //int maxLevel = 2;
    TermCriteria criteria(TermCriteria::COUNT | TermCriteria::EPS, 10, 0.03);
    // PRINT("DUNNO");
    // calculate optical flow
    vector<Point2f> points_new;
    vector<uchar> status;
    vector<float> error;
    calcOpticalFlowPyrLK(prev_bgr, bgr, points_old, points_new, status, error, winSize, maxLevel, criteria);
    //flow->error = error;

    
    // filter the flow vector by their status
    vector<Point2f> good_points_old, good_points_new, flow_vectors;
    vector<float> error_new;
    for (size_t i = 0; i < status.size(); i++) {
        if (status[i] == 1) {
            good_points_old.push_back(points_old[i]);
            good_points_new.push_back(points_new[i]);
            flow_vectors.push_back(points_new[i] - points_old[i]);
            error_new.push_back(error[i]);
        }
    }
    // PRINT("BEFOREFILTER");
    // filter the flow vectors by their magnitude
    vector<float> magnitudes;
    for (size_t i = 0; i < flow_vectors.size(); i++) {
        // PRINT("MAG");
        magnitudes.push_back(sqrt(pow(flow_vectors[i].x, 2) + pow(flow_vectors[i].y, 2)));
    }

    // PRINT("ALLLEEE");
    // find the flow vectors that are above the threshold and save them
    // float threshold = 0.7 * (float)(*max_element(magnitudes.begin(), magnitudes.end()));
    
    // // PRINT("THREHSEERROR");
    // vector<Point2f> good_flow_vectors;
    // vector<float> good_error;

    // lin_vectors.resize(good_points_old.size());

    // // PRINT("RESIZEERROR");
    // for (size_t i = 0; i < flow_vectors.size(); i++) {
        

    //     if (magnitudes[i] >= threshold) {
    //         good_flow_vectors.push_back(flow_vectors[i]);
    //         good_points_old.push_back(good_points_old[i]);
    //         good_points_new.push_back(good_points_new[i]);

       
    //         // PRINT("FORLOOPERROR");
    //         lin_vectors[i].pos.x = good_points_old[i].x;
    //         lin_vectors[i].pos.y = good_points_old[i].y;

            
    //         lin_vectors[i].flow_x = flow_vectors[i].x;
    //         lin_vectors[i].flow_y = flow_vectors[i].y;
            
    //         lin_vectors[i].error = error_new[i];
    //         lin_vectors[i].pos.count = 0;
    //         lin_vectors[i].pos.x_sub = 0;
    //         lin_vectors[i].pos.y_sub = 0;
    //     }
    // }
    // sort magnitudes in descending order
    // PRINT("BEFORESORT");
    sort(magnitudes.rbegin(), magnitudes.rend());

    // find the flow vectors that are above the threshold and save them
    float threshold = 0.0;
    int num_top_vectors = (int)(0.2 * flow_vectors.size()); // calculate number of top vectors
    int num_nonzero_vectors = 0;

    for (size_t i = 0; i < flow_vectors.size(); i++) {
        if (magnitudes[i] > 0) {
            num_nonzero_vectors++;
            if (num_nonzero_vectors <= num_top_vectors) {
                threshold = magnitudes[i];
            }
        }
    }
    // PRINT("AFTERSORT");
    vector<Point2f> good_flow_vectors;
    vector<float> good_error;
    lin_vectors.resize(good_points_old.size());

    for (size_t i = 0; i < flow_vectors.size(); i++) {
        if (magnitudes[i] >= threshold && good_flow_vectors.size() < num_top_vectors) {
            good_flow_vectors.push_back(flow_vectors[i]);
            good_points_old.push_back(good_points_old[i]);
            good_points_new.push_back(good_points_new[i]);

            lin_vectors[i].pos.x = good_points_old[i].x;
            lin_vectors[i].pos.y = good_points_old[i].y;

            lin_vectors[i].flow_x = flow_vectors[i].x;
            lin_vectors[i].flow_y = flow_vectors[i].y;

            lin_vectors[i].error = error_new[i];
            lin_vectors[i].pos.count = 0;
            lin_vectors[i].pos.x_sub = 0;
            lin_vectors[i].pos.y_sub = 0;
        }
    }

//    PRINT("SERGGGGGG");
    
    // Declare an array of flow_t with the same size as lin_vectors
    flow_t* flow_array = new flow_t[lin_vectors.size()];
    int count_array = 0;

    // Loop through each element in lin_vectors and copy its data into the corresponding element in flow_array
    for (size_t i = 0; i < lin_vectors.size(); i++) {
            flow_array[i].pos = lin_vectors[i].pos;
            flow_array[i].flow_x = lin_vectors[i].flow_x;
            flow_array[i].flow_y = lin_vectors[i].flow_y;
            flow_array[i].error = lin_vectors[i].error;
            count_array++;
            // if(i==50){
            //     break;}

            
            //flow_array[0].pos.count = i;

    }
    // cout << "Count of array:";
    // cout << count_array;

    *array_size = count_array;
    //return flow_array;

    // PRINT("YAMAC SUCKS");
    // return result_analyzer;
    return flow_array;

}