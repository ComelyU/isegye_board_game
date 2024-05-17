package com.accio.isegye.game.dto;

import com.accio.isegye.game.entity.Stock;
import java.time.LocalDateTime;

public record StockResponse(
    Integer id,
    Integer storeId,
    String storeName,
    GameResponse game,
    Integer isAvailable,
    String stockLocation,
    LocalDateTime createdAt,
    LocalDateTime updatedAt,
    LocalDateTime deletedAt
) {
    public StockResponse(Stock stock) {
        this(
            stock.getId(),
            stock.getStore().getId(),
            stock.getStore().getStoreName(),
            new GameResponse(stock.getGame()),
            stock.getIsAvailable(),
            stock.getStockLocation(),
            stock.getCreatedAt(),
            stock.getUpdatedAt(),
            stock.getDeletedAt()
        );
    }
}
