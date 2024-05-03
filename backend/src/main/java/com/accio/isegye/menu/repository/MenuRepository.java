package com.accio.isegye.menu.repository;

import com.accio.isegye.menu.dto.MenuResponse;
import com.accio.isegye.menu.entity.Menu;
import java.util.List;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface MenuRepository extends JpaRepository<Menu, Integer> {

    List<Menu> findByStoreIdAndDeletedAtIsNull(int storeId);
}
