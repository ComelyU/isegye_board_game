package com.accio.isegye.store.entity;

import com.accio.isegye.common.entity.BaseTimeEntity;
import com.accio.isegye.menu.entity.Menu;
import com.accio.isegye.turtle.entity.Turtle;
import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.EntityListeners;
import jakarta.persistence.FetchType;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import jakarta.persistence.Id;
import jakarta.persistence.OneToMany;
import java.util.ArrayList;
import java.util.List;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

import lombok.experimental.SuperBuilder;
import org.springframework.data.jpa.domain.support.AuditingEntityListener;

@Getter
@Entity
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@EntityListeners(AuditingEntityListener.class)
@SuperBuilder
public class Store extends BaseTimeEntity {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "id", insertable = false, updatable = false)
    private int id;

    private String storeName;
    private int hourFee;

    //터틀봇 정보
    @OneToMany(mappedBy = "store", orphanRemoval = true, fetch = FetchType.LAZY)
    private final List<Turtle> turtleBotList = new ArrayList<>();

    //방
    @OneToMany(mappedBy = "store", orphanRemoval = true, fetch = FetchType.LAZY)
    private final List<Room> roomList = new ArrayList<>();

    //메뉴
    @OneToMany(mappedBy = "store", orphanRemoval = true, fetch = FetchType.LAZY)
    private final List<Menu> menuList = new ArrayList<>();

    //보드게임 재고
//    @OneToMany(mappedBy = "store", fetch = FetchType.LAZY)
//    private final List<Stock> stockList = new ArrayList<>();

}
