package com.example.domain.usecase

import com.example.domain.model.BasicResponseClass
import com.example.domain.repository.Repository
import javax.inject.Inject

class GameCancelUseCase @Inject constructor(
    private val repository: Repository,
) {
    suspend operator fun invoke(gameOrderId: Int): Result<BasicResponseClass> =
        repository.cancelGameRepo(gameOrderId)
}
