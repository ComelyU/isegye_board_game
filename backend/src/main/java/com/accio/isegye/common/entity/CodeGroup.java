package com.accio.isegye.common.entity;

import com.accio.isegye.game.entity.GameTagCategory;
import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.EntityListeners;
import jakarta.persistence.FetchType;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import jakarta.persistence.Id;
import jakarta.persistence.OneToMany;
import jakarta.persistence.OneToOne;
import java.util.ArrayList;
import java.util.List;
import lombok.AccessLevel;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.experimental.SuperBuilder;
import org.springframework.data.jpa.domain.support.AuditingEntityListener;

@Getter
@Entity
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@AllArgsConstructor(access = AccessLevel.PROTECTED)
@EntityListeners(AuditingEntityListener.class)
@Builder
public class CodeGroup {
    @Id
    @Column(name = "group_name", updatable = false)
    private String groupName;

    private String groupDescription;

    @OneToMany(mappedBy = "codeGroup", fetch = FetchType.LAZY)
    private final List<CodeItem> codeItemList = new ArrayList<>();

    public void updateGroupDescription(String description) {
        this.groupDescription = description;
    }
}
