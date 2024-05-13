package com.accio.isegye.config;

import org.springframework.context.annotation.Configuration;
import org.springframework.messaging.simp.config.MessageBrokerRegistry;
import org.springframework.web.socket.config.annotation.EnableWebSocketMessageBroker;
import org.springframework.web.socket.config.annotation.StompEndpointRegistry;
import org.springframework.web.socket.config.annotation.WebSocketMessageBrokerConfigurer;

@Configuration
@EnableWebSocketMessageBroker
public class WebSocketConfig implements WebSocketMessageBrokerConfigurer {

    @Override
    public void configureMessageBroker(MessageBrokerRegistry registry) {
        // 메모리 기반 메시지 브로커를 사용하여 클라이언트로 메시지를 전송하는데 사용될 prefix를 설정
        registry.enableSimpleBroker("/topic");
        // 클라이언트에서 메시지를 보낼 때 사용될 prefix를 설정
        registry.setApplicationDestinationPrefixes("/app");
    }

    @Override
    public void registerStompEndpoints(StompEndpointRegistry registry) {
        // WebSocket endpoint를 "/kafka-messages"로 설정
        registry.addEndpoint("/kafka-messages")
            .setAllowedOrigins("https://k10a706.p.ssafy.io", "http://localhost:3000")
            .withSockJS();
    }
}
