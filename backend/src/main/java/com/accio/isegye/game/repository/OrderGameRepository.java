package com.accio.isegye.game.repository;

import com.accio.isegye.game.entity.OrderGame;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface OrderGameRepository extends JpaRepository<OrderGame, Long> {

}
