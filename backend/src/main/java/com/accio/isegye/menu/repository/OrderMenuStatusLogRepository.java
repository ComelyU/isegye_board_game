package com.accio.isegye.menu.repository;

import com.accio.isegye.menu.entity.OrderMenuStatusLog;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface OrderMenuStatusLogRepository extends JpaRepository<OrderMenuStatusLog, Long> {

}
