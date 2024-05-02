package com.accio.isegye.game.service;

import com.accio.isegye.game.repository.GameRepository;
import com.accio.isegye.game.repository.GameTagCategoryRepository;
import com.accio.isegye.game.repository.OrderGameRepository;
import com.accio.isegye.game.repository.OrderGameStatusLogRepository;
import com.accio.isegye.game.repository.StockRepository;
import com.accio.isegye.game.repository.ThemeRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class GameServiceImpl implements GameService{

    private final GameRepository gameRepository;
    private final GameTagCategoryRepository categoryRepository;
    private final OrderGameRepository orderRepository;
    private final OrderGameStatusLogRepository logRepository;
    private final StockRepository stockRepository;
    private final ThemeRepository themeRepository;


}
