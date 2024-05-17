package com.accio.isegye.common.repository;

import com.accio.isegye.common.entity.CodeItem;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface CodeItemRepository extends JpaRepository<CodeItem, Integer> {

}
