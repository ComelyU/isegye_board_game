package com.accio.isegye.common.entity;

import jakarta.annotation.Nullable;
import jakarta.persistence.Column;
import jakarta.persistence.EntityListeners;
import jakarta.persistence.MappedSuperclass;
import java.time.LocalDateTime;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.experimental.SuperBuilder;
import org.springframework.data.annotation.CreatedDate;
import org.springframework.data.annotation.LastModifiedDate;
import org.springframework.data.jpa.domain.support.AuditingEntityListener;

@EntityListeners(AuditingEntityListener.class)
@Getter
@SuperBuilder
@MappedSuperclass
@NoArgsConstructor
public class BaseTimeEntity {

    @CreatedDate
    @Column(updatable = false)
    private LocalDateTime createdAt;

    @LastModifiedDate
    private LocalDateTime updatedAt;

    @Nullable
    private LocalDateTime deletedAt;

    /**
     * deletedAt 필드 값을 현재 시간으로 설정하여 엔티티를 '삭제된 상태'로 표시.
     */
    protected void markAsDeleted() {
        this.deletedAt = LocalDateTime.now();
    }
}
