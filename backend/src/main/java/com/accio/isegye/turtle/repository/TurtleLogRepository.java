package com.accio.isegye.turtle.repository;

import com.accio.isegye.store.entity.Room;
import com.accio.isegye.turtle.entity.TurtleLog;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface TurtleLogRepository extends JpaRepository<TurtleLog, Integer> {

}
