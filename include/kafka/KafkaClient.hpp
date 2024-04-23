#include <stdio.h>
#include <string>

#include <librdkafka/rdkafkacpp.h>

#ifndef __VSTK_KAFKA_CLIENT_H
#define __VSTK_KAFKA_CLIENT_H

namespace vstk {
    class KafkaProducer {
        private:
            const std::string IMAGE_TOPIC = "t_image";
            const std::string IMAGE_CTX_TOPIC = "t_image_ctx";
            const std::string MATCH_TOPIC = "t_matches";

            std::string kafka_addr;

            RdKafka::Conf *global_conf = RdKafka::Conf::create(RdKafka::Conf::CONF_GLOBAL);
            RdKafka::Conf *topic_conf = RdKafka::Conf::create(RdKafka::Conf::CONF_TOPIC);

            RdKafka::Producer *producer = nullptr;
            RdKafka::Topic *image_topic = NULL;

        public:
            KafkaProducer(std::string kafka_addr);
            void publish_match_event(std::string data);
    };

    class KafkaConsumer {
        private:
            std::string kafka_addr;
            rd_kafka_conf_t *conf = rd_kafka_conf_new();

            rd_kafka_t *consumer = nullptr;
        public:
            KafkaConsumer(std::string kafka_addr);
            void consume_match_event(std::string data);
    };


    class KafkaClient {
        private:
            std::string kafka_addr;
            rd_kafka_conf_t *producer_conf = rd_kafka_conf_new();
            rd_kafka_conf_t *consumer_conf = rd_kafka_conf_new();

            rd_kafka_t *producer = nullptr;
            rd_kafka_t *consumer = nullptr;
        public:
            KafkaClient(std::string kafka_addr);
            void publish_match_event(std::string data);
    };
}

#endif