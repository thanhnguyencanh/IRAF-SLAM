/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2024-2025 Thanh Nguyen Canh, Bao Quoc Nguyen, HaoLan Zhang, Xiem HoangVan, and Chong NakYoung, School of Information Science, Japan Advanced Institute of Science and Technology (JAIST).
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

#include <vector>
#include <cmath>
#include <numeric>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

// Calculates the normalized histogram of a grayscale image.
cv::Mat calculate_histogram(const cv::Mat& image);
cv::Mat adjust_gamma(const cv::Mat& gray_image, const std::vector<float>& gamma_values);
cv::Mat create_contrast_mask(const cv::Mat& original_image, const cv::Mat& adjusted_image);
cv::Mat adaptive_gamma_adjustment(const cv::Mat& image, float alpha = 0.2, float tau = 0.5);
std::pair<cv::Mat, cv::Mat> unsharp_mask(const cv::Mat& image, float alpha = 1.0f);
cv::Mat image_agcwd(const cv::Mat& img, double a = 0.25, bool truncated_cdf = false);
cv::Mat process_bright(const cv::Mat& img);
cv::Mat process_dimmed(const cv::Mat& img);
/**
 * @brief Processes an image by adjusting its brightness based on a specified threshold and expected mean value.
 * 
 * This function analyzes the brightness of the input image and applies adjustments to ensure the brightness
 * is closer to the expected mean value. It is useful for normalizing image brightness in computer vision tasks.
 * 
 * @param img The input image to be processed. It should be a valid cv::Mat object.
 * @param threshold A double value representing the threshold for brightness adjustment. Default is 0.3.
 *                  This value determines the sensitivity of the adjustment.
 * @param expectedMean A double value representing the expected mean brightness of the image. Default is 112.0.
 *                     The function adjusts the image brightness to approach this mean value.
 * @return cv::Mat The processed image with adjusted brightness.
 */
cv::Mat process_image_based_on_brightness(const cv::Mat& img, double threshold = 0.3, double expectedMean = 112.0);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    double t_resize = 0.f;
    double t_track = 0.f;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        

        // tag-change
        cv::Mat adjusted_image = process_image_based_on_brightness(im, 0.3, 50.0);
        cv::Mat contrast_mask =  create_contrast_mask(im,adjusted_image);
        im = unsharp_mask(adjusted_image, 1.5).first;


        double tframe = vTimestamps[ni];


        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}

/**
 * @brief Calculates the normalized histogram of an input image.
 * 
 * This function computes the histogram of a single-channel image and normalizes it
 * so that the sum of all bins equals 1. The histogram represents the distribution
 * of pixel intensity values in the image.
 * 
 * @param image The input image (single-channel, e.g., grayscale) for which the histogram is calculated.
 *              It must be of type CV_8U (8-bit unsigned integer).
 * 
 * @return cv::Mat A 1D matrix (vector) representing the normalized histogram of the input image.
 *                 The size of the matrix is equal to the number of bins (256 by default).
 * 
 * @note The function assumes the input image is a grayscale image with intensity values
 *       in the range [0, 255]. If the input image has a different type or range, the behavior
 *       may be undefined.
 * 
 * @warning The input image must not be empty. Passing an empty image will result in an error.
 */
cv::Mat calculate_histogram(const cv::Mat& image) {
        cv::Mat hist;

        int histSize = 256;                // Number of bins
        float range[] = {0.0f, 256.0f};    // Range of intensity values
        const float* histRange = range;    // Pointer to the range array

        // Calculate the histogram of the image
        cv::calcHist(&image, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);

        // Normalize the histogram to make the sum of all bins equal to 1
        hist /= cv::sum(hist)[0];

        return hist;
        }

        cv::Mat adjust_gamma(const cv::Mat& gray_image, const std::vector<float>& gamma_values) {
        cv::Mat adjusted_image = cv::Mat::zeros(gray_image.size(), gray_image.type());

        for (int i = 0; i < gray_image.rows; ++i) {
                for (int j = 0; j < gray_image.cols; ++j) {
                uchar pixel_value = gray_image.at<uchar>(i, j);
                float gamma = gamma_values[pixel_value];
                cv::Vec3b pixel = gray_image.at<cv::Vec3b>(i, j);
                adjusted_image.at<cv::Vec3b>(i, j) = cv::Vec3b(
                        cv::saturate_cast<uchar>(255 * std::pow(pixel[0] / 255.0f, gamma)),
                        cv::saturate_cast<uchar>(255 * std::pow(pixel[1] / 255.0f, gamma)),
                        cv::saturate_cast<uchar>(255 * std::pow(pixel[2] / 255.0f, gamma))
                );
                }
        }
        return adjusted_image;
        }

        cv::Mat create_contrast_mask(const cv::Mat& original_image, const cv::Mat& adjusted_image) {
        cv::Mat contrast_mask;
        cv::subtract(adjusted_image, original_image, contrast_mask);
        return contrast_mask;
        }


        std::pair<cv::Mat, cv::Mat> unsharp_mask(const cv::Mat& image, float alpha) {
        cv::Mat blurred, Gmask, I_unsharpened;
        cv::GaussianBlur(image, blurred, cv::Size(5, 5), 0);
        cv::subtract(image, blurred, Gmask);
        cv::addWeighted(image, 1.0, Gmask, alpha, 0, I_unsharpened);
        return {I_unsharpened, Gmask};
        }


        // Function to calculate cumulative sum
        cv::Mat calculate_cumulative_sum(const cv::Mat& hist) {
        cv::Mat cdf(hist.size(), hist.type(), cv::Scalar(0));
        float* histData = (float*)hist.data;
        float* cdfData = (float*)cdf.data;

        cdfData[0] = histData[0];
        for (int i = 1; i < hist.rows; ++i) {
                cdfData[i] = cdfData[i - 1] + histData[i];
        }
        return cdf;
        }

        // AGCWD function
        cv::Mat image_agcwd(const cv::Mat& img, double a, bool truncated_cdf ) {
        CV_Assert(img.type() == CV_8UC1); // Input must be grayscale

        // Compute histogram
        int histSize = 256;
        float range[] = {0, 256};
        const float* histRange = {range};
        cv::Mat hist;
        cv::calcHist(&img, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, false);
        hist /= img.total(); // Normalize histogram

        // Compute cumulative distribution function (CDF)
        cv::Mat cdf = calculate_cumulative_sum(hist);

        // Compute probability normalization
        double probMin = *std::min_element(hist.begin<float>(), hist.end<float>());
        double probMax = *std::max_element(hist.begin<float>(), hist.end<float>());
        cv::Mat probNorm = (hist - probMin) / (probMax - probMin);

        // Apply weighted distortion
        for (int i = 0; i < probNorm.rows; ++i) {
                float& val = probNorm.at<float>(i);
                if (val > 0) {
                val = probMax * std::pow(val, a);
                } else if (val < 0) {
                val = probMax * -std::pow(-val, a);
                }
        }
        probNorm /= cv::sum(probNorm)[0]; // Normalize to [0, 1]
        cv::Mat cdfProbNorm = calculate_cumulative_sum(probNorm);

        // Compute inverse CDF
        cv::Mat inverseCDF;
        if (truncated_cdf) {
                cv::max(0.5, 1 - cdfProbNorm, inverseCDF);
        } else {
                inverseCDF = 1 - cdfProbNorm;
        }

        // Apply AGCWD to the image
        cv::Mat result = img.clone();
        for (int i = 0; i < 256; ++i) {
                result.setTo(cv::Scalar(std::round(255 * std::pow(i / 255.0, inverseCDF.at<float>(i)))), img == i);
        }

        return result;
        }

        // Bright image processing
        cv::Mat process_bright(const cv::Mat& img) {
        cv::Mat imgNegative = 255 - img;
        cv::Mat agcwd = image_agcwd(imgNegative, 0.25, false);
        return 255 - agcwd;
        }

        // Dimmed image processing
        cv::Mat process_dimmed(const cv::Mat& img) {
        return image_agcwd(img, 0.75, true);
        }

        // Function to determine if the image is bright or dimmed and process it
        cv::Mat process_image_based_on_brightness(const cv::Mat& img, double threshold, double expectedMean ) {
        // Calculate mean intensity of the image
        double meanIntensity = cv::mean(img)[0];
        double t = (meanIntensity - expectedMean) / expectedMean;

        // Process image based on its brightness
        if (t < -threshold) {
                return process_dimmed(img);
        } else if (t > threshold) {
                return process_bright(img);
        } else {
                return img;
        }
        }

        cv::Mat adaptive_gamma_adjustment(const cv::Mat& image, float alpha , float tau ) {
        cv::Mat gray_image;
        if (image.channels() == 1) {  
                gray_image = image;
        } else {
                cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY); 
        }

        cv::Mat hist = calculate_histogram(gray_image);

        int n = gray_image.total();
        std::vector<float> P_i(hist.rows);
        for (int i = 0; i < hist.rows; ++i) {
                P_i[i] = hist.at<float>(i, 0) * (1.0f / n);
        }

        float P_max = *std::max_element(P_i.begin(), P_i.end());

        float P_min = *std::min_element(P_i.begin(), P_i.end());

        std::vector<float> P_w(P_i.size());

        for (size_t i = 0; i < P_i.size(); ++i) {
                P_w[i] = P_max * std::pow((P_i[i] - P_min) / (P_max - P_min), alpha);
        }

        std::vector<float> C_w(P_w.size());
        std::partial_sum(P_w.begin(), P_w.end(), C_w.begin());
        float total_Pw = std::accumulate(P_w.begin(), P_w.end(), 0.0f);
        for (float& cw : C_w) {
                cw /= total_Pw;
        }

        std::vector<float> gamma_values(C_w.size());
        for (size_t i = 0; i < C_w.size(); ++i) {
                gamma_values[i] = std::max(tau, 1.0f - C_w[i]);
        }
        cv::Mat adjusted_image = adjust_gamma(gray_image, gamma_values);

        return adjusted_image;
        }