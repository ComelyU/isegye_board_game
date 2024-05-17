package com.accio.isegye.game.repository;

import com.accio.isegye.game.dto.GameResponse;
import com.accio.isegye.game.entity.Game;
import java.util.Arrays;
import java.util.List;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.stereotype.Repository;

@Repository
public interface GameRepository extends JpaRepository<Game, Integer> {

    boolean existsByIdAndDeletedAtIsNull(int gameId);

    List<Game> findByIdIn(List<Integer> body);

    @Query(
        "select game\n"
          + "from GameTagCategory gtc\n"
          + "left join gtc.game game\n"
          + "left join gtc.codeItem ci\n"
          + "where game.maxPlaytime >= ?1 and game.maxPlaytime <= ?2\n"
          + "and game.gameDifficulty >= ?3 and game.gameDifficulty <= ?4\n"
          + "and game.theme.themeType like ?5\n"
          + "and ci.itemName like ?6")
    List<Game> findByFilter(
        Integer minPlaytime,
        Integer maxPlaytime,
        Float minDifficulty,
        Float maxDifficulty,
        String theme,
        String tag);

    @Query(
        "select game\n"
          + "from GameTagCategory gtc\n"
          + "left join gtc.game game\n"
          + "left join gtc.codeItem ci\n"
          + "where ci.itemName like ?1")
    List<Game> findByCategory(String tag);

    @Query(
        "select game\n"
            + "from GameTagCategory gtc\n"
            + "left join gtc.game game\n"
            + "left join gtc.codeItem ci\n"
            + "where game.theme.themeType like ?1")
    List<Game> findByTheme(String theme);
}
