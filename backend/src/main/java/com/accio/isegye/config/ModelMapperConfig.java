package com.accio.isegye.config;

import com.accio.isegye.exception.CustomException;
import com.accio.isegye.exception.ErrorCode;
import com.accio.isegye.game.dto.StockResponse;
import com.accio.isegye.game.entity.Stock;
import java.util.Collections;
import java.util.stream.Collectors;
import org.modelmapper.ModelMapper;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.modelmapper.config.Configuration.AccessLevel;

@Configuration
public class ModelMapperConfig {

    @Bean
    public ModelMapper modelMapper() {
        ModelMapper modelMapper = new ModelMapper();
        modelMapper.getConfiguration()
            // private 이어도 접근할 수 있음
            .setFieldAccessLevel(AccessLevel.PRIVATE)
            // 필드명 같으면 자동 매핑 처리
            .setFieldMatchingEnabled(true);

//        modelMapper.typeMap(Stock.class, StockResponse.class).addMappings(mapper -> {
//           mapper.map(src -> {
//                   if (src.getGame() != null && src.getGame().getCategoryList() != null) {
//                       return src.getGame().getCategoryList()
//                           .stream()
//                           .map(gameTagCategory -> gameTagCategory.getCodeItem().getItemName())
//                           .collect(Collectors.toList());
//                   }
//                   return Collections.emptyList();
//               },
//                StockResponse::setCodeItemName
//           );
//        });

        return modelMapper;
    }
}
