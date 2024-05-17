package com.example.domain.usecase

import com.example.domain.model.RemoteResponseClass
import com.example.domain.repository.Repository
import javax.inject.Inject

class GameCancelUseCase @Inject constructor(
    private val repository: Repository,
) {
    suspend operator fun invoke(gameOrderId: Int): Result<RemoteResponseClass> =
        repository.cancelGameRepo(gameOrderId)
}
