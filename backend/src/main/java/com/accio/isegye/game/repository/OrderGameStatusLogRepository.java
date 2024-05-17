package com.accio.isegye.game.repository;

import com.accio.isegye.game.entity.OrderGameStatusLog;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface OrderGameStatusLogRepository extends JpaRepository<OrderGameStatusLog, Long> {

}
