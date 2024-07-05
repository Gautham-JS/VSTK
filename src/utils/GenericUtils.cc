#include "utils/GenericUtils.hpp"

using namespace vstk;


static std::random_device               rd;
static std::mt19937                     gen(rd());
static std::uniform_int_distribution<>  dis(0, 15);
static std::uniform_int_distribution<>  dis2(8, 11);

std::string vstk::generate_random_string(uint8_t length) {
    const std::string characters = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    std::uniform_int_distribution<> dis(0, characters.size() - 1);
    std::string randomString;
    randomString.reserve(length);
    for (int i = 0; i < length; ++i) {
        randomString += characters[dis(gen)];
    }
    return randomString;
}

std::string vstk::generate_uuid() {
    std::stringstream ss;
    int i;
    ss << std::hex;
    for (i = 0; i < 8; i++) {
        ss << dis(gen);
    }
    ss << "-";
    for (i = 0; i < 4; i++) {
        ss << dis(gen);
    }
    ss << "-4";
    for (i = 0; i < 3; i++) {
        ss << dis(gen);
    }
    ss << "-";
    ss << dis2(gen);
    for (i = 0; i < 3; i++) {
        ss << dis(gen);
    }
    ss << "-";
    for (i = 0; i < 12; i++) {
        ss << dis(gen);
    };
    return ss.str();
}