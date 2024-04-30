package com.accio.isegye.mqtt;

import lombok.Getter;
import lombok.Setter;
import org.springframework.boot.context.properties.ConfigurationProperties;
import org.springframework.boot.context.properties.bind.ConstructorBinding;


@Setter
@Getter
@ConfigurationProperties(prefix = "spring.mqtt")
public class MqttProperties {
    private final String brokerUrl;
    private final String brokerClientId;
    private final String topicFilter;
    private final String username;
    private final String password;
    private final Integer keepAliveInterval;

    @ConstructorBinding
    public MqttProperties(String brokerUrl, String brokerClientId, String topicFilter,
        String username,
        String password, Integer keepAliveInterval) {
        this.brokerUrl = brokerUrl;
        this.brokerClientId = brokerClientId;
        this.topicFilter = topicFilter;
        this.username = username;
        this.password = password;
        this.keepAliveInterval = keepAliveInterval;
    }
}
