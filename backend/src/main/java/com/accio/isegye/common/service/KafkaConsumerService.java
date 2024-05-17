package com.accio.isegye.common.service;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.kafka.annotation.KafkaListener;
import org.springframework.messaging.simp.SimpMessagingTemplate;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
@Slf4j
public class KafkaConsumerService {

    private final SimpMessagingTemplate template;

    @KafkaListener(topics = {"accio-isegye"}, groupId = "accio-isegye")
    public void consume(String message) {
        log.info("Kafka Consume = topic: {}, message: {}", "accio-isegye", message);
    }

    @KafkaListener(topics = "OrderGame", groupId = "accio-isegye")
    public void consumeOrderGame(String message) {
        log.info("Kafka Consume = topic: {}, message: {}", "OrderGame", message);

        // 받은 메시지를 WebScoket을 통해 전송
        template.convertAndSend("/topic/OrderGame", message);
    }

    @KafkaListener(topics = "OrderMenu", groupId = "accio-isegye")
    public void consumeOrderMenu(String message) {
        log.info("Kafka Consume = topic: {}, message: {}", "OrderMenu", message);

        // 받은 메시지를 WebScoket을 통해 전송
        template.convertAndSend("/topic/OrderMenu", message);
    }
}
