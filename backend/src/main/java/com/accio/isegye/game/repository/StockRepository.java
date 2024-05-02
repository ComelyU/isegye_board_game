package com.accio.isegye.game.repository;

import com.accio.isegye.game.dto.StockResponse;
import com.accio.isegye.game.entity.Stock;
import java.util.List;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface StockRepository extends JpaRepository<Stock, Integer> {

    List<Stock> findAllByStoreId(int storeId);
}
