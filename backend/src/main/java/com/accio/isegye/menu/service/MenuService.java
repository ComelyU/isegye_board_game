package com.accio.isegye.menu.service;

import com.accio.isegye.menu.dto.CreateMenuRequest;
import com.accio.isegye.menu.dto.CreateOrderMenuRequest;
import com.accio.isegye.menu.dto.MenuResponse;
import com.accio.isegye.menu.dto.OrderMenuResponse;
import com.accio.isegye.menu.dto.UpdateMenuRequest;
import com.accio.isegye.menu.entity.Menu;
import java.util.List;

public interface MenuService {

    MenuResponse createMenu(int storeId, CreateMenuRequest request);

    List<MenuResponse> getMenuList(int storeId);

    MenuResponse updateMenu(int menuId, UpdateMenuRequest request);

    void deleteMenu(int menuId);

    OrderMenuResponse createOrderMenu(List<CreateOrderMenuRequest> orderMenuRequest, int customerId);

    List<OrderMenuResponse> getOrderMenu(int customerId);

    List<OrderMenuResponse> getStoreOrderMenu(int storeId);

    void readyOrderMenu(long orderMenuId);

    void turtleOrderMenu(long orderMenuId);

    void completeOrderMenu(long orderMenuId);
}
