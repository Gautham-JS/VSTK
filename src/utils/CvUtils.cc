#include "utils/CvUtils.hpp"
#include "utils/Logger.hpp"

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

float vstk::normalize_float(float value, float min_range, float max_range) {
    // Check if value is within the specified range
    if (value < min_range || value > max_range) {
        std::cerr << "Error: Value is outside the specified range." << std::endl;
        return -1; // Return an error code
    }
    // Calculate the normalized value
    return ((float)(value - min_range) / (max_range - min_range)) * 255.0;
}

void vstk::write_ply(std::vector<cv::Point3f> points, std::string filename) {
    std::vector<cv::Point3f> clipped_pts;
    vstk::clip_pcl(points, clipped_pts, 0, 100);
    std::ofstream outfile(filename);
    outfile << "ply\n" << "format ascii 1.0\n" << "comment VTK generated PLY File\n";
    outfile << "obj_info vtkPolyData points and polygons : vtk4.0\n" << "element vertex " << clipped_pts.size() << "\n";
    outfile << "property float x\n" << "property float y\n" << "property float z\n" << "element face 0\n";
    outfile << "property list uchar int vertex_indices\n" << "end_header\n";
    for (int i = 0; i < clipped_pts.size(); i++) {
        cv::Point3d point = clipped_pts.at(i);
        outfile << point.x << " ";
        outfile << point.y << " ";
        outfile << point.z << " ";
        outfile << "\n";
    }
    outfile.close();
}

void vstk::draw_depth(cv::Mat image, std::vector<cv::Point2f> p2d, std::vector<cv::Point3f> p3d) {
    cv::Mat depths;
    std::vector<uchar> data;
    std::vector<bool> clipping_idxs(p2d.size(), false);

    float min_positive_depth = (float) INT32_MAX;
    float max_positive_depth = 0.0f;
    float min_clip_dist = 0.0f;
    float max_clip_dist = 100.0f;

    for(cv::Point3f pt3d : p3d) {
        //std::cerr << pt3d << std::endl;
        if(pt3d.z < min_clip_dist || pt3d.z > max_clip_dist) continue;
        if(pt3d.z < min_positive_depth) min_positive_depth = pt3d.z;
        if(pt3d.z > max_positive_depth) max_positive_depth = pt3d.z;
    }
    //DBGLOG("MIN_DEPTH : %f, MAX_DEPTH : %f", min_positive_depth, max_positive_depth);
    for(size_t i=0; i<p2d.size(); i++) {
        float depth = p3d[i].z;
        if(depth < min_clip_dist || depth > max_clip_dist) {
            depth = min_positive_depth;
            clipping_idxs[i] = true;
        }
        float norm_depth = normalize_float(depth, min_positive_depth, max_positive_depth);
        //DBGLOG("DEPTH : %f, NORM_DEPTH : %f", depth, norm_depth);
        data.push_back(norm_depth);
    }
    cv::Mat depth_vector(data);
    cv::Mat depth_vector_2d = depth_vector.reshape(1, data.size());
    cv::Mat depth_mat;
    cv::Mat depth_im = image.clone();
    cv::applyColorMap(depth_vector_2d, depth_mat, cv::COLORMAP_JET);
    for(size_t i=0; i<p2d.size(); i++) {
        cv::Vec3b &color = depth_mat.at<cv::Vec3b>(0, i);
        if(clipping_idxs[i]) {
            continue;
        }
        cv::drawMarker(depth_im, p2d[i], color, cv::MARKER_CROSS, 20, 2);
    }
    cv::resize(depth_im, depth_im, {depth_im.cols/2, depth_im.rows/2});
    cv::imshow("Depth", depth_im);
    int key = (cv::waitKey(1) & 0xFF);
    if(key == 'q') {
        cv::destroyAllWindows();
        exit(0);
    }
}

void vstk::draw_reprojection(cv::Mat image, cv::Mat P, std::vector<cv::Point2f> p2d, cv::Mat p4d) {
    for(size_t i=0; i<p4d.cols; i++){
        cv::Mat col4d;
        p4d.col(i).convertTo(col4d, P.type());
        cv::Mat pt2d_reprojection = P * col4d;
        DBGLOG("2D Point : (%f, %f)", p2d[i].x, p2d[i].y);
        DBGLOG("Reprojection : (%f, %f)", col4d.at<double>(0, 0), col4d.at<double>(1, 0));
    }
}


void vstk::clip_pcl(std::vector<cv::Point3f> pts, std::vector<cv::Point3f> &out_pts, int min_clip_depth, int max_clip_depth) {
    out_pts.clear();
    out_pts.reserve(pts.size());
    for(size_t i=0; i<pts.size(); i++) {
        float depth = pts[i].z;
        if(depth < min_clip_depth || depth > max_clip_depth) {
            continue;
        }
        out_pts.push_back(pts[i]);
    }
}