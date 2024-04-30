#include "utils/CvUtils.hpp"


std::vector<cv::Mat> vstk::split_image( cv::Mat & image, int M, int N ) {
    std::vector<std::pair<int, int>> origins;
    return split_image(image, M, N, origins);
}


std::vector<cv::Mat> vstk::split_image( cv::Mat & image, int M, int N , std::vector<std::pair<int, int>> &origins) {
    // All images should be the same size ...
    int width  = image.cols / M;
    int height = image.rows / N;
    // ... except for the Mth column and the Nth row
    int width_last_column = width  + ( image.cols % width  );
    int height_last_row   = height + ( image.rows % height );
    origins.clear();
    std::vector<cv::Mat> result;
    for( int i = 0; i < N; ++i ) {
        for( int j = 0; j < M; ++j ) {
            cv::Rect roi( 
                width  * j,
                height * i,
                ( j == ( M - 1 ) ) ? width_last_column : width,
                ( i == ( N - 1 ) ) ? height_last_row   : height 
            );
            origins.emplace_back(std::make_pair(width * j, height * i));
            result.push_back( image( roi ) );
        }
    }
    return result;
}