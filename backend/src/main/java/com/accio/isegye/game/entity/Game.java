package com.accio.isegye.game.entity;

import com.accio.isegye.common.entity.BaseTimeEntity;
import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.EntityListeners;
import jakarta.persistence.FetchType;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import jakarta.persistence.Id;
import jakarta.persistence.JoinColumn;
import jakarta.persistence.ManyToOne;
import jakarta.persistence.OneToMany;
import java.util.ArrayList;
import java.util.List;
import lombok.AccessLevel;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.experimental.SuperBuilder;
import org.springframework.data.jpa.domain.support.AuditingEntityListener;

@Getter
@Entity
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@EntityListeners(AuditingEntityListener.class)
@SuperBuilder
public class Game extends BaseTimeEntity {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "id", insertable = false, updatable = false)
    private int id;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "theme_id")
    private Theme theme;

    private String gameName;
    private String gameDetail;
    private int minPlayer;
    private int maxPlayer;
    private int minPlaytime;
    private int maxPlaytime;
    private float gameDifficulty;
    private String gameImgUrl;

    @OneToMany(mappedBy = "game", fetch = FetchType.LAZY)
    private final List<Stock> stockList = new ArrayList<>();

    @OneToMany(mappedBy = "game", fetch = FetchType.LAZY)
    private final List<GameTagCategory> categoryList = new ArrayList<>();

}
