package com.accio.isegye.rabbitmq;

import com.accio.isegye.config.MqttConfig.MqttGateway;
import com.accio.isegye.turtle.dto.TurtleOrderRequest;
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

    @RequestMapping(value = "/send/mqtt", method = RequestMethod.POST)
    public ResponseEntity<?> sendMqtt(@RequestBody String mqttMessage) {
//        JsonObject convertObject = new Gson().fromJson(mqttMessage, JsonObject.class);
//        mqttGateway.sendToMqtt(convertObject.get("content").toString(), "ros_test");
        mqttGateway.sendToMqtt(mqttMessage, "display/1");
        return ResponseEntity.ok("Message sent to MQTT!");
    }

    @RequestMapping(value = "/send/order", method = RequestMethod.POST)
    public ResponseEntity<?> sendReceiveOrder(@RequestBody TurtleOrderRequest request) {
//        JsonObject convertObject = new Gson().fromJson(mqttMessage, JsonObject.class);
        mqttGateway.sendToMqtt(request.toString(), "server/"+request.getTurtleId());
        return ResponseEntity.ok("Message sent to MQTT!");
    }

}
