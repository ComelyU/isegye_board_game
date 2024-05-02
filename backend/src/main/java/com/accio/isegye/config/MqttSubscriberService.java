package com.accio.isegye.config;

import java.util.Arrays;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.springframework.stereotype.Service;

@Service
public class MqttSubscriberService implements MqttCallback {
    private MqttClient mqttClient;

    //커넥션 종료 (통신오류로 연결이 끊어지는 경우) 호출
    @Override
    public void connectionLost(final Throwable throwable) {
        System.out.println("Connection is lost");
        System.out.println(throwable);
    }

    //메시지 도착 시 호출
    @Override
    public void messageArrived(String topic, MqttMessage mqttMessage) throws Exception {
        StringBuilder sb = new StringBuilder();
        sb.append("Message Arrived").append("\n");
        sb.append(mqttMessage).append("\n");
        sb.append("topic = ").append(topic).append("\n");
        sb.append("id = ").append(mqttMessage.getId()).append("\n");
        sb.append("payload = ").append(Arrays.toString(mqttMessage.getPayload()));

        System.out.println(sb.toString());
    }

    //구독 신청
    public boolean subscribe(final String topic) throws MqttException{
        if(topic!= null){
            mqttClient.subscribe(topic, 1);
        }

        return true;
    }

    // 메시지의 배달이 완료되면 호출
    @Override
    public void deliveryComplete(IMqttDeliveryToken iMqttDeliveryToken) {

    }
}
