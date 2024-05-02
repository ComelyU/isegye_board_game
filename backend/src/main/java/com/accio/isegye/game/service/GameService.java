package com.accio.isegye.game.service;

import com.accio.isegye.game.dto.GameResponse;
import com.accio.isegye.game.dto.StockResponse;
import java.util.List;

public interface GameService {

    List<StockResponse> getStockList(int storeId);

    List<GameResponse> getGameList(int gameId);
}
