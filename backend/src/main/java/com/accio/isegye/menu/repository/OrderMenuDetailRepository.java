package com.accio.isegye.menu.repository;

import com.accio.isegye.menu.entity.OrderMenuDetail;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface OrderMenuDetailRepository extends JpaRepository<OrderMenuDetail, Long> {

}
