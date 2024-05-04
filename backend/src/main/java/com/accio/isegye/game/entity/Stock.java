package com.accio.isegye.game.entity;

import com.accio.isegye.common.entity.BaseTimeEntity;
import com.accio.isegye.store.entity.Store;
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
public class Stock extends BaseTimeEntity {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "id", insertable = false, updatable = false)
    private int id;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "store_id")
    private Store store;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "game_id")
    private Game game;

    private Integer isAvailable;

    private String stockLocation;

    @OneToMany(mappedBy = "stock", fetch = FetchType.LAZY)
    private final List<OrderGame> orderGameList = new ArrayList<>();

    public void updateIsAvailableAndStockLocation(Integer isAvailable, String stockLocation) {
        this.isAvailable = isAvailable;
        this.stockLocation = stockLocation;
    }

    public void softDelete() {
        super.markAsDeleted();
    }
}
