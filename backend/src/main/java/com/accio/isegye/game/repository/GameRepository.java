package com.accio.isegye.game.repository;

import com.accio.isegye.game.dto.GameResponse;
import com.accio.isegye.game.entity.Game;
import java.util.List;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface GameRepository extends JpaRepository<Game, Integer> {

    boolean existsByIdAndDeletedAtIsNull(int gameId);

    List<Game> findByIdIn(List<Integer> body);
}
