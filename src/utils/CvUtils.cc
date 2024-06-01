#include "utils/CvUtils.hpp"
#include <fstream>


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

void vstk::join_image_in_another(
    cv::Mat &parent, 
    cv::Mat child, 
    std::pair<int, int> child_origin_in_parent
    ) {
    child.copyTo(parent(cv::Rect(child_origin_in_parent.first, child_origin_in_parent.second, child.cols, child.rows)));
}


int vstk::normalize(int value, int min_range, int max_range) {
    // Check if value is within the specified range
    if (value < min_range || value > max_range) {
        std::cerr << "Error: Value is outside the specified range." << std::endl;
        return -1; // Return an error code
    }
    // Calculate the normalized value
    double normalized_value = ((double)(value - min_range) / (max_range - min_range)) * 255.0;
    // Round the normalized value to the nearest integer
    int result = (int)(normalized_value + 0.5);
    return result;
}

void vstk::write_ply(std::vector<cv::Point3f> points, std::string filename) {
    std::ofstream outfile(filename);
    outfile << "ply\n" << "format ascii 1.0\n" << "comment VTK generated PLY File\n";
    outfile << "obj_info vtkPolyData points and polygons : vtk4.0\n" << "element vertex " << points.size() << "\n";
    outfile << "property float x\n" << "property float y\n" << "property float z\n" << "element face 0\n";
    outfile << "property list uchar int vertex_indices\n" << "end_header\n";
    for (int i = 0; i < points.size(); i++) {
        cv::Point3d point = points.at(i);
        outfile << point.x << " ";
        outfile << point.y << " ";
        outfile << point.z << " ";
        outfile << "\n";
    }
    outfile.close();
}