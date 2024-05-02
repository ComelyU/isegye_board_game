package com.accio.isegye.game.service;

import com.accio.isegye.game.dto.GameResponse;
import com.accio.isegye.game.dto.StockResponse;
import com.accio.isegye.game.entity.Game;
import com.accio.isegye.game.entity.Stock;
import com.accio.isegye.game.repository.GameRepository;
import com.accio.isegye.game.repository.GameTagCategoryRepository;
import com.accio.isegye.game.repository.OrderGameRepository;
import com.accio.isegye.game.repository.OrderGameStatusLogRepository;
import com.accio.isegye.game.repository.StockRepository;
import com.accio.isegye.game.repository.ThemeRepository;
import com.accio.isegye.store.dto.StoreResponse;
import com.accio.isegye.store.entity.Store;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.modelmapper.ModelMapper;
import org.springframework.stereotype.Service;

@Slf4j
@Service
@RequiredArgsConstructor
public class GameServiceImpl implements GameService{

    private final GameRepository gameRepository;
    private final GameTagCategoryRepository categoryRepository;
    private final OrderGameRepository orderRepository;
    private final OrderGameStatusLogRepository logRepository;
    private final StockRepository stockRepository;
    private final ThemeRepository themeRepository;
    private final ModelMapper modelMapper;

    // Stock과 StockResponse 매핑
    private StockResponse getStockResponse(Stock stock){
        return modelMapper.map(stock, StockResponse.class);
    }

    @Override
    public List<StockResponse> getStockList(int storeId) {

        List<StockResponse> stockResponse = stockRepository.findAllByStoreId (storeId)
            .stream()
            .map(this::getStockResponse)
            .toList();

        return stockResponse;
    }

    public List<GameResponse> getGameList(int gameId){
        return gameRepository.findById(gameId).stream().map(game -> modelMapper.map(game, GameResponse.class)).toList();
    }

}
