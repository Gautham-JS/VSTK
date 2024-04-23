#include "kafka/KafkaClient.hpp"
#include "utils/Logger.hpp"

vstk::KafkaProducer::KafkaProducer(std::string kafka_addr) : kafka_addr(kafka_addr) {
    std::string err;
    global_conf->set("metadata.broker.list", kafka_addr, err);
    producer = RdKafka::Producer::create(global_conf, err);
    if (!producer) {
        ERRORLOG("Failed to create Kafka producer : %s", err);
        exit(1);
    }
    
    image_topic = RdKafka::Topic::create(producer, IMAGE_CTX_TOPIC, topic_conf, err);
    if(!image_topic) {
        ERRORLOG("Failed to create Kafka image topic : %s", err);
        exit(1);
    }
}



void vstk::KafkaProducer::publish_match_event(std::string data) {
    INFOLOG("Publishing match data event");

}