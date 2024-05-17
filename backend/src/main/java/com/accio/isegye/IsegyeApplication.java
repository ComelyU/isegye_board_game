package com.accio.isegye;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.boot.context.properties.EnableConfigurationProperties;
import org.springframework.data.jpa.repository.config.EnableJpaAuditing;

@SpringBootApplication
@EnableJpaAuditing
public class IsegyeApplication {

    public static void main(String[] args) {
        SpringApplication.run(IsegyeApplication.class, args);
    }

}
