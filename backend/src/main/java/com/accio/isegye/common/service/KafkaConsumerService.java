package com.accio.isegye.common.service;

import lombok.extern.slf4j.Slf4j;
import org.springframework.kafka.annotation.KafkaListener;
import org.springframework.stereotype.Service;

@Service
@Slf4j
public class KafkaConsumerService {

    @KafkaListener(topics = {"accio-isegye"}, groupId = "accio-isegye")
    public void consume(String message) {
        log.info("Kafka Consume = topic: {}, message: {}", "accio-isegye", message);
    }
}
