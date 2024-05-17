package com.example.domain.usecase

import com.example.domain.model.RemoteResponseClass
import com.example.domain.repository.Repository
import javax.inject.Inject

class MenuCancelUseCase @Inject constructor(
    private val repository: Repository,
) {
    suspend operator fun invoke(menuOrderId: Int): Result<RemoteResponseClass> =
        repository.cancelMenuRepo(menuOrderId)
}