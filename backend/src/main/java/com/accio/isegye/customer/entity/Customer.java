package com.accio.isegye.customer.entity;

import com.accio.isegye.game.entity.OrderGame;
import com.accio.isegye.menu.entity.OrderMenu;
import com.accio.isegye.store.entity.Room;
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
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;
import lombok.AccessLevel;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;
import lombok.experimental.SuperBuilder;
import org.springframework.data.annotation.CreatedDate;
import org.springframework.data.jpa.domain.support.AuditingEntityListener;

@Getter
@Setter
@Entity
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@EntityListeners(AuditingEntityListener.class)
@SuperBuilder
public class Customer {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "id", insertable = false, updatable = false)
    private int id;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name="room_id")
    private Room room;

    private int isTheme;

    private int peopleNum;

    @CreatedDate
    @Column(updatable = false)
    private LocalDateTime startTime;

    private LocalDateTime endTime;

    private int roomFee;

    //주문 메뉴
    @OneToMany(mappedBy = "customer", fetch = FetchType.LAZY)
    private final List<OrderMenu> orderMenuList = new ArrayList<>();

    //주문 게임
    @OneToMany(mappedBy = "customer", fetch = FetchType.LAZY)
    private final List<OrderGame> orderGameList = new ArrayList<>();

}
