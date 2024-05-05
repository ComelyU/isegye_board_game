package com.accio.isegye.game.dto;

import com.accio.isegye.game.entity.OrderGame;
import java.time.LocalDateTime;

public record OrderGameResponse(
    Long id,
    Integer customerId,
    Integer stockId,
    String gameName,
    String stockLocation,
    Integer orderType,
    Integer orderStatus,
    LocalDateTime createdAt,
    LocalDateTime deliveredAt
) {
    public OrderGameResponse(OrderGame orderGame) {
        this(
            orderGame.getId(),
            orderGame.getCustomer().getId(),
            orderGame.getStock().getId(),
            orderGame.getStock().getGame().getGameName(),
            orderGame.getStock().getStockLocation(),
            orderGame.getOrderType(),
            orderGame.getOrderStatus(),
            orderGame.getCreatedAt(),
            orderGame.getDeliveredAt()
        );
    }
}
