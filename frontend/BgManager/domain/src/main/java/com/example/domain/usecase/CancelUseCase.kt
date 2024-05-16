package com.example.domain.usecase

import com.example.domain.model.RemoteResponseClass
import com.example.domain.repository.Repository
import javax.inject.Inject

class CancelUseCase @Inject constructor(
    private val repository: Repository,
) {
    suspend operator fun invoke(gameOrderId: Int): Result<RemoteResponseClass> =
        repository.cancelRepo(gameOrderId)
}
