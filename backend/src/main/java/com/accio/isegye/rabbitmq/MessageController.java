package com.accio.isegye.rabbitmq;

import com.accio.isegye.config.MqttConfig.MqttGateway;
import com.google.gson.Gson;
import com.google.gson.JsonObject;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.amqp.rabbit.annotation.RabbitListener;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestMethod;
import org.springframework.web.bind.annotation.RestController;

@Slf4j
@RequiredArgsConstructor
@RestController
public class MessageController {

    @Autowired
    private final MessageService messageService;
    @Autowired
    private MqttGateway mqttGateway;

//
//    /**
//     * Queue로 메시지를 발행
//     *
//     * @param messageDto 발행할 메시지의 DTO 객체
//     * @return ResponseEntity 객체로 응답을 반환
//     */
//    @RequestMapping(value = "/send/message", method = RequestMethod.POST)
//    public ResponseEntity<?> sendMessage(@RequestBody MessageDto messageDto) {
//        messageService.sendMessage(messageDto);
//        return ResponseEntity.ok("Message sent to RabbitMQ!");
//    }

    @RequestMapping(value = "/send/mqtt", method = RequestMethod.POST)
    public ResponseEntity<?> sendMqtt(@RequestBody String mqttMessage) {
        JsonObject convertObject = new Gson().fromJson(mqttMessage, JsonObject.class);
        mqttGateway.sendToMqtt(convertObject.get("content").toString(), "isegye.key");
        return ResponseEntity.ok("Message sent to MQTT!");
    }

}
