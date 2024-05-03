package com.accio.isegye.menu.repository;

import com.accio.isegye.menu.entity.Menu;
import com.accio.isegye.menu.entity.OrderMenu;
import java.util.List;
import org.hibernate.query.Order;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface OrderMenuRepository  extends JpaRepository<OrderMenu, Long> {

    List<OrderMenu> findAllByCustomerId(int customerId);

    List<OrderMenu> findAllByCustomerRoomStoreIdAndOrderStatusLessThanEqual(int storeId, int orderStatus);
}
