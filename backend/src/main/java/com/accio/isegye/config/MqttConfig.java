package com.accio.isegye.config;

import com.accio.isegye.mqtt.MqttProperties;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.integration.annotation.MessagingGateway;
import org.springframework.integration.annotation.ServiceActivator;
import org.springframework.integration.channel.DirectChannel;
import org.springframework.integration.core.MessageProducer;
import org.springframework.integration.mqtt.core.DefaultMqttPahoClientFactory;
import org.springframework.integration.mqtt.core.MqttPahoClientFactory;
import org.springframework.integration.mqtt.inbound.MqttPahoMessageDrivenChannelAdapter;
import org.springframework.integration.mqtt.outbound.MqttPahoMessageHandler;
import org.springframework.integration.mqtt.support.DefaultPahoMessageConverter;
import org.springframework.integration.mqtt.support.MqttHeaders;
import org.springframework.messaging.Message;
import org.springframework.messaging.MessageChannel;
import org.springframework.messaging.MessageHandler;
import org.springframework.messaging.MessagingException;
import org.springframework.messaging.handler.annotation.Header;
import org.springframework.stereotype.Component;

@Configuration
@RequiredArgsConstructor
@Component
@Slf4j
public class MqttConfig {
    @Autowired
    private final MqttProperties mqttProperties;

    private MqttConnectOptions connectOptions(){
        MqttConnectOptions options = new MqttConnectOptions();
        options.setUserName(mqttProperties.getUsername());
        options.setPassword(mqttProperties.getPassword().toCharArray());
        options.setKeepAliveInterval(mqttProperties.getKeepAliveInterval());
        options.setServerURIs(new String[]{mqttProperties.getBrokerUrl()});
        return options;

    }

    @Bean
    public MqttPahoClientFactory MqttClientFactory(){
        DefaultMqttPahoClientFactory clientFactory = new DefaultMqttPahoClientFactory();
        clientFactory.setConnectionOptions(connectOptions());

        return clientFactory;
    }
    
    /*
    * 수신
    * */

    @Bean
    public MessageChannel mqttInputChannel(){
        return new DirectChannel();
    }

    @Bean
    public MessageProducer inboundChannel(){
        MqttPahoMessageDrivenChannelAdapter adapter =
            new MqttPahoMessageDrivenChannelAdapter(mqttProperties.getBrokerClientId(), MqttClientFactory(), "#");

        adapter.setCompletionTimeout(5000);
        adapter.setConverter(new DefaultPahoMessageConverter());
        adapter.setQos(1);
        adapter.setOutputChannel(mqttInputChannel());

        return adapter;
    }

    @Bean
    @ServiceActivator(inputChannel = "mqttInputChannel")
    public MessageHandler handler(){
        return new MessageHandler() {
            @Override
            public void handleMessage(Message<?> message) throws MessagingException {
                String topic = message.getHeaders().get(MqttHeaders.RECEIVED_TOPIC).toString();
                if (topic.equals(mqttProperties.getTopicFilter())) {
                    System.out.println("this is our topic");
                }else{
                    System.out.println("new topic: " + topic);
                }
                System.out.println(message.getPayload());
            }
        };
    }
    
    /*
    * 송신
    * */

    @Bean
    public MessageChannel mqttOutboundChannel(){
        return new DirectChannel();
    }

    @Bean
    @ServiceActivator(inputChannel = "mqttOutboundChannel")
    public MessageHandler mqttOutbound(){
        MqttPahoMessageHandler messageHandler =
            new MqttPahoMessageHandler(mqttProperties.getBrokerServerId(), MqttClientFactory());
        messageHandler.setAsync(true);
//        messageHandler.setDefaultQos(1);
        messageHandler.setDefaultTopic("#");

        return messageHandler;
    }

}
