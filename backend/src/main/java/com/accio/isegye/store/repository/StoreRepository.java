package com.accio.isegye.store.repository;

import com.accio.isegye.store.dto.StoreResponse;
import com.accio.isegye.store.entity.Store;
import java.util.List;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.stereotype.Repository;

@Repository
public interface StoreRepository extends JpaRepository<Store, Integer> {

    List<Store> findAllByDeletedAtIsNull();

    @Query("select s.hourFee from Store s where s.id=?1")
    int findHourFeeById(int id);
}
