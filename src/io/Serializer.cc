#include "io/Serializer.hpp"



vstk::Serializer::Serializer() {

}



std::string vstk::Serializer::stringify_match(const cv::DMatch match) {
    std::stringstream ss;
    ss << match.distance << DELIM << match.imgIdx << DELIM << match.queryIdx << DELIM <<match.trainIdx;
    return ss.str();
}

/*
SER STRUCT : 
x;
y;
response;
octave;
angle;
size;
class_id;
*/
std::string vstk::Serializer::stringify_kp(const cv::KeyPoint kp) {
    std::stringstream ss;
    ss << kp.pt.x << DELIM;
    ss << kp.pt.y << DELIM;
    ss << kp.response << DELIM;
    ss << kp.octave << DELIM;
    ss << kp.angle << DELIM;
    ss << kp.size << DELIM;
    ss << kp.class_id << DELIM;
    return ss.str();
}


/*
SER STRUCT : 
image_id;
kps_size;
{kp1;kp2;kp3....};
desc_size;
{d1;d2;d3;....};
*/
vstk::BinaryDataStream* vstk::Serializer::serialize(ImageContextHolder image_ctx) {
    INFOLOG("Serializing image...");
    std::stringstream ss;
    ss << image_ctx.get_image_id() << DELIM;
    ss << image_ctx.get_features_holder().kps.size() << DELIM;
    for(int i=0; i<image_ctx.get_features_holder().kps.size(); i++) {
        cv::KeyPoint kp = image_ctx.get_features_holder().kps[i];
        ss << stringify_kp(kp);
    }
    cv::Mat desc = image_ctx.get_features_holder().descriptors;
    ss << desc.size().height << DELIM;
    ss << desc.size().width << DELIM;
    desc = desc.reshape(0, 1);
    ss << desc << DELIM;

    std::string data_str = ss.str();
    BinaryDataStream *data_stream = new BinaryDataStream();
    data_stream->data = data_str;
    data_stream->data_size = data_str.size();
    return data_stream;

}

vstk::BinaryDataStream* vstk::Serializer::serialize(MatchesHolder mholder) {
    std::stringstream ss;
    ss << mholder.im1_id << DELIM;
    ss << mholder.im2_id << DELIM;

    ss << mholder.good_matches.size() << DELIM;
    for(int i=0; i<mholder.good_matches.size(); i++) {
        cv::DMatch match = mholder.good_matches[i];
        ss << stringify_match(match);
    }
    std::string data_str = ss.str();
    BinaryDataStream *data_stream = new BinaryDataStream();
    data_stream->data = data_str;
    data_stream->data_size = data_str.size();
    return data_stream;
}