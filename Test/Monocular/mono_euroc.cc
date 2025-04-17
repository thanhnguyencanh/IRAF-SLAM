/**
 * This file is part of ORB-SLAM3
 *
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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <cmath>
#include <numeric>


#include <System.h>

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

cv::Mat calculate_histogram(const cv::Mat& image);
cv::Mat adjust_gamma(const cv::Mat& gray_image, const std::vector<float>& gamma_values);
cv::Mat create_contrast_mask(const cv::Mat& original_image, const cv::Mat& adjusted_image);
cv::Mat adaptive_gamma_adjustment(const cv::Mat& image, float alpha = 0.2, float tau = 0.5);
std::pair<cv::Mat, cv::Mat> unsharp_mask(const cv::Mat& image, float alpha = 1.0f);
cv::Mat image_agcwd(const cv::Mat& img, double a = 0.25, bool truncated_cdf = false);
cv::Mat process_bright(const cv::Mat& img);
cv::Mat process_dimmed(const cv::Mat& img);
cv::Mat process_image_based_on_brightness(const cv::Mat& img, double threshold = 0.3, double expectedMean = 112.0);

int main(int argc, char **argv)
{
    if (argc < 5)
    {
        cerr << endl
             << "Usage: ./mono_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) (trajectory_file_name)" << endl;
        return 1;
    }

    const int num_seq = (argc - 3) / 2;
    cout << "num_seq = " << num_seq << endl;
    bool bFileName = (((argc - 3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc - 1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq;
    vector<vector<string>> vstrImageFilenames;
    vector<vector<double>> vTimestampsCam;
    vector<int> nImages;

    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    nImages.resize(num_seq);

    int tot_images = 0;
    for (seq = 0; seq < num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";
        LoadImages(string(argv[(2 * seq) + 3]) + "/mav0/cam0/data", string(argv[(2 * seq) + 4]), vstrImageFilenames[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageFilenames[seq].size();
        tot_images += nImages[seq];
    }
    // cout << "Number of images: " << tot_images.size() << endl;
    

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl
         << "-------" << endl;
    cout.precision(17);

    int fps = 20;
    float dT = 1.f / fps;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, false);
    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track = 0.f;
   

    for (seq = 0; seq < num_seq; seq++)
    {

        // Main loop
        cv::Mat im;
        int proccIm = 0;
        for (int ni = 0; ni < nImages[seq]; ni++, proccIm++)
        {

            // Read image from file
            im = cv::imread(vstrImageFilenames[seq][ni], cv::IMREAD_GRAYSCALE); //,CV_LOAD_IMAGE_UNCHANGED);

            // tag-change
            // cv::Mat adjusted_image= adaptive_gamma_adjustment(im, 1.9, 0.5);
            // cv::Mat adjusted_image= gamma_result.first;
            // cv::Mat sharpened_image = unsharp_mask(adjusted_image, 1.5).first;
            // im=sharpened_image;

            cv::Mat adjusted_image = process_image_based_on_brightness(im, 0.3, 50.0);
            cv::Mat contrast_mask =  create_contrast_mask(im,adjusted_image);
            im = unsharp_mask(adjusted_image, 1.5).first;


            double tframe = vTimestampsCam[seq][ni];



            if (im.empty())
            {
                cerr << endl
                     << "Failed to load image at: "
                     << vstrImageFilenames[seq][ni] << endl;

                return 1;
            }

            if (imageScale != 1.f)
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
                t_resize = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_End_Resize - t_Start_Resize).count();
                SLAM.InsertResizeTime(t_resize);
#endif
            }

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

            // Pass the image to the SLAM system
            // cout << "tframe = " << tframe << endl;
            SLAM.TrackMonocular(im, tframe); // TODO change to monocular_inertial

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

            vTimesTrack[ni] = ttrack;

            // Wait to load the next frame
            double T = 0;
            if (ni < nImages[seq] - 1)
                T = vTimestampsCam[seq][ni + 1] - tframe;
            else if (ni > 0)
                T = tframe - vTimestampsCam[seq][ni - 1];

            // std::cout << "T: " << T << std::endl;
            // std::cout << "ttrack: " << ttrack << std::endl;

            if (ttrack < T)
            {
                // std::cout << "usleep: " << (dT-ttrack) << std::endl;
                usleep((T - ttrack) * 1e6); // 1e6
            }
        }

        if (seq < num_seq - 1)
        {
            string kf_file_submap = "./SubMaps/kf_SubMap_" + std::to_string(seq) + ".txt";
            string f_file_submap = "./SubMaps/f_SubMap_" + std::to_string(seq) + ".txt";
            SLAM.SaveTrajectoryEuRoC(f_file_submap);
            SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file_submap);

            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }
    }
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file = "results/kf_" + string(argv[argc - 1]) + ".txt";
        const string f_file = "results/f_" + string(argv[argc - 1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t * 1e-9);
        }
    }

}

cv::Mat calculate_histogram(const cv::Mat& image) {
    cv::Mat hist;

    int histSize = 256;                
    float range[] = {0.0f, 256.0f};   
    const float* histRange = range;    

    cv::calcHist(&image, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);

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