package com.accio.isegye.turtle.entity;

import com.accio.isegye.game.entity.OrderGame;
import com.accio.isegye.menu.entity.OrderMenu;
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
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;
import lombok.AccessLevel;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;
import lombok.ToString;
import lombok.experimental.SuperBuilder;
import org.springframework.data.annotation.CreatedDate;
import org.springframework.data.jpa.domain.support.AuditingEntityListener;

@Getter
@Setter
@Entity
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@EntityListeners(AuditingEntityListener.class)
@SuperBuilder
public class TurtleLog {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "id", insertable = false, updatable = false)
    private long id;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name="turtle_id")
    private Turtle turtle;

    //목적지
    private int commandType;

    @CreatedDate
    @Column(updatable = false)
    private LocalDateTime commandStartTime;

    private LocalDateTime commandEndTime;

    private int isSuccess;

    //JSON의 형태
    private String logMessage;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "order_menu_id")
    private OrderMenu orderMenu;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "order_game_id")
    private OrderGame orderGame;

    @Override
    public String toString() {
        return "TurtleLog{" +
            "id=" + id +
            ", turtle=" + turtle +
            ", commandType=" + commandType +
            ", commandStartTime=" + commandStartTime +
            ", commandEndTime=" + commandEndTime +
            ", isSuccess=" + isSuccess +
            ", logMessage='" + logMessage + '\'' +
            ", orderMenu=" + orderMenu +
            ", orderGame=" + orderGame +
            '}';
    }
}
